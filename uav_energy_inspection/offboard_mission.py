#!/usr/bin/env python3
"""PX4 SITL mission: arm, take off to 2 m, hover, and land."""

from enum import Enum, auto
from math import nan

import rclpy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class MissionState(Enum):
    WAITING_FOR_PX4 = auto()
    STREAMING_SETPOINTS = auto()
    TAKING_OFF = auto()
    HOVERING = auto()
    LANDING = auto()
    COMPLETE = auto()


class OffboardMission(Node):
    """Run a guarded, simulation-only takeoff/hover/land sequence."""

    TAKEOFF_ALTITUDE_NED = -2.0
    HOVER_SECONDS = 8.0
    SETPOINT_WARMUP_CYCLES = 20

    def __init__(self) -> None:
        super().__init__('offboard_mission')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            qos,
        )
        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self.status_callback,
            qos,
        )

        self.position = None
        self.status = None
        self.state = MissionState.WAITING_FOR_PX4
        self.warmup_cycles = 0
        self.hover_started_ns = None
        self.last_wait_log_ns = 0
        self.preflight_warning_logged = False
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            'Milestone 2 ready: waiting for PX4 position and vehicle status...')

    def now_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def position_callback(self, message: VehicleLocalPosition) -> None:
        self.position = message

    def status_callback(self, message: VehicleStatus) -> None:
        self.status = message

    def publish_offboard_heartbeat(self) -> None:
        message = OffboardControlMode()
        message.timestamp = self.now_us()
        message.position = True
        message.velocity = False
        message.acceleration = False
        message.attitude = False
        message.body_rate = False
        message.thrust_and_torque = False
        message.direct_actuator = False
        self.offboard_publisher.publish(message)

    def publish_takeoff_setpoint(self) -> None:
        message = TrajectorySetpoint()
        message.timestamp = self.now_us()
        message.position = [0.0, 0.0, self.TAKEOFF_ALTITUDE_NED]
        message.velocity = [nan, nan, nan]
        message.acceleration = [nan, nan, nan]
        message.jerk = [nan, nan, nan]
        message.yaw = 0.0
        message.yawspeed = nan
        self.setpoint_publisher.publish(message)

    def publish_vehicle_command(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
    ) -> None:
        message = VehicleCommand()
        message.timestamp = self.now_us()
        message.param1 = param1
        message.param2 = param2
        message.command = command
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.confirmation = 0
        message.from_external = True
        self.command_publisher.publish(message)

    def start_offboard_flight(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )
        self.state = MissionState.TAKING_OFF
        self.get_logger().info('OFFBOARD and arm commands sent; taking off to 2.0 m.')

    def request_landing(self) -> None:
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.state = MissionState.LANDING
        self.get_logger().info('Hover complete; automatic landing requested.')

    def px4_is_ready(self) -> bool:
        telemetry_ready = (
            self.position is not None
            and self.status is not None
            and self.position.xy_valid
            and self.position.z_valid
        )
        if (
            telemetry_ready
            and not self.status.pre_flight_checks_pass
            and not self.preflight_warning_logged
        ):
            self.get_logger().warning(
                'PX4 reports a preflight warning (expected without QGroundControl); '
                'PX4 will still enforce all arming checks when the command is sent.')
            self.preflight_warning_logged = True
        return telemetry_ready

    def log_waiting_periodically(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_wait_log_ns >= 5_000_000_000:
            self.get_logger().info(
                'Still waiting: start PX4 SITL and MicroXRCEAgent if they are not running.')
            self.last_wait_log_ns = now_ns

    def timer_callback(self) -> None:
        if self.state == MissionState.COMPLETE:
            return

        if self.state == MissionState.WAITING_FOR_PX4:
            if not self.px4_is_ready():
                self.log_waiting_periodically()
                return
            self.state = MissionState.STREAMING_SETPOINTS
            self.get_logger().info(
                'PX4 is ready; warming up the offboard setpoint stream.')

        if self.state in (
            MissionState.STREAMING_SETPOINTS,
            MissionState.TAKING_OFF,
            MissionState.HOVERING,
        ):
            self.publish_offboard_heartbeat()
            self.publish_takeoff_setpoint()

        if self.state == MissionState.STREAMING_SETPOINTS:
            self.warmup_cycles += 1
            if self.warmup_cycles >= self.SETPOINT_WARMUP_CYCLES:
                self.start_offboard_flight()
            return

        if self.state == MissionState.TAKING_OFF:
            if (
                self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
                and self.position.z <= -1.8
            ):
                self.state = MissionState.HOVERING
                self.hover_started_ns = self.get_clock().now().nanoseconds
                self.get_logger().info('Reached 2.0 m; hovering for 8 seconds.')
            return

        if self.state == MissionState.HOVERING:
            elapsed = (
                self.get_clock().now().nanoseconds - self.hover_started_ns
            ) / 1_000_000_000
            if elapsed >= self.HOVER_SECONDS:
                self.request_landing()
            return

        if self.state == MissionState.LANDING:
            if self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.state = MissionState.COMPLETE
                self.get_logger().info(
                    'Mission complete: vehicle landed and disarmed safely.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OffboardMission()
    try:
        while rclpy.ok() and node.state != MissionState.COMPLETE:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.get_logger().info('Mission node stopped by operator.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
