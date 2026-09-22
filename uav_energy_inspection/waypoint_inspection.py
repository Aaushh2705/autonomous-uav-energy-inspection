#!/usr/bin/env python3
"""Autonomous PX4 SITL inspection mission for the substation world."""

import csv
from datetime import datetime
from enum import Enum, auto
from html import escape
from math import nan, sqrt
from pathlib import Path
from threading import Lock

import cv2
import numpy as np
import rclpy
from gz.msgs10.image_pb2 import Image, PixelFormatType
from gz.transport13 import Node as GazeboTransportNode
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class MissionState(Enum):
    WAITING_FOR_PX4 = auto()
    STREAMING_SETPOINTS = auto()
    TAKING_OFF = auto()
    INSPECTING = auto()
    RETURNING = auto()
    LANDING = auto()
    COMPLETE = auto()


class WaypointInspection(Node):
    """Visit five substation assets, return home, and land."""

    CRUISE_ALTITUDE_NED = -10.0
    SETPOINT_WARMUP_CYCLES = 20
    ARRIVAL_RADIUS_M = 1.3
    ALTITUDE_TOLERANCE_M = 1.0
    INSPECTION_HOLD_SECONDS = 3.0
    MAX_MISSION_SECONDS = 240.0

    # PX4 local NED coordinates mapped from the yellow ENU world markers.
    WAYPOINTS = [
        ('Transformer A west', -12.0, 28.0, -10.0, 1.5708),
        ('Transformer B west', 12.0, 28.0, -10.0, 1.5708),
        ('Transformer A east', -12.0, 47.0, -10.0, -1.5708),
        ('Transformer B east', 12.0, 47.0, -10.0, -1.5708),
        # View the wide gantry broadside rather than along its narrow end.
        ('High-voltage gantry', 0.0, 50.0, -18.0, -1.5708),
    ]
    CAMERA_TOPIC = (
        '/world/energy_substation/model/inspection_x500_0/link/'
        'camera_link/sensor/inspection_camera/image'
    )

    def __init__(self) -> None:
        super().__init__('waypoint_inspection')

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
        self.create_subscription(
            VehicleLandDetected,
            '/fmu/out/vehicle_land_detected',
            self.land_detected_callback,
            qos,
        )

        self.position = None
        self.status = None
        self.land_detected = None
        self.state = MissionState.WAITING_FOR_PX4
        self.target = (0.0, 0.0, self.CRUISE_ALTITUDE_NED)
        self.target_yaw = 0.0
        self.warmup_cycles = 0
        self.waypoint_index = 0
        self.hold_started_ns = None
        self.arrival_logged = False
        self.mission_started_ns = None
        self.last_wait_log_ns = 0
        self.preflight_warning_logged = False
        self.native_landing = False
        self.disarm_sent = False
        self.latest_camera_frame = None
        self.camera_lock = Lock()
        self.camera_warning_logged = False
        self.inspection_records = []
        self.report_written = False

        log_directory = Path(
            '/home/aayush1/uav_ws/src/uav_energy_inspection/data')
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.run_directory = log_directory / f'inspection_mission_{timestamp}'
        self.image_directory = self.run_directory / 'images'
        self.image_directory.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.run_directory / 'inspection_data.csv'
        self.report_path = self.run_directory / 'inspection_report.html'
        with self.csv_path.open('w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow([
                'inspection_time',
                'asset',
                'target_north_m',
                'target_east_m',
                'target_down_m',
                'measured_north_m',
                'measured_east_m',
                'measured_down_m',
                'camera_status',
                'image_file',
            ])

        self.gz_node = GazeboTransportNode()
        camera_subscribed = self.gz_node.subscribe(
            Image, self.CAMERA_TOPIC, self.camera_callback)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(
            'Milestone 3 ready: waiting for PX4 telemetry...')
        self.get_logger().info(f'Inspection output: {self.run_directory}')
        if camera_subscribed:
            self.get_logger().info('Gazebo inspection camera connected.')
        else:
            self.get_logger().warning(
                'Gazebo camera subscription failed; flight can continue without images.')

    def now_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def position_callback(self, message: VehicleLocalPosition) -> None:
        self.position = message

    def status_callback(self, message: VehicleStatus) -> None:
        self.status = message

    def land_detected_callback(self, message: VehicleLandDetected) -> None:
        self.land_detected = message

    def camera_callback(self, message: Image) -> None:
        channels_by_format = {
            PixelFormatType.RGB_INT8: (3, cv2.COLOR_RGB2BGR),
            PixelFormatType.BGR_INT8: (3, None),
            PixelFormatType.RGBA_INT8: (4, cv2.COLOR_RGBA2BGR),
            PixelFormatType.BGRA_INT8: (4, cv2.COLOR_BGRA2BGR),
            PixelFormatType.L_INT8: (1, None),
        }
        format_details = channels_by_format.get(message.pixel_format_type)
        if format_details is None:
            if not self.camera_warning_logged:
                self.get_logger().warning(
                    f'Unsupported camera pixel format: {message.pixel_format_type}')
                self.camera_warning_logged = True
            return

        channels, conversion = format_details
        expected_row_bytes = message.width * channels
        row_bytes = message.step if message.step else expected_row_bytes
        raw = np.frombuffer(message.data, dtype=np.uint8)
        required_bytes = message.height * row_bytes
        if raw.size < required_bytes:
            return
        rows = raw[:required_bytes].reshape(message.height, row_bytes)
        pixels = rows[:, :expected_row_bytes]
        if channels == 1:
            frame = pixels.reshape(message.height, message.width)
        else:
            frame = pixels.reshape(message.height, message.width, channels)
        if conversion is not None:
            frame = cv2.cvtColor(frame, conversion)
        with self.camera_lock:
            self.latest_camera_frame = frame.copy()

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

    def publish_target(self) -> None:
        message = TrajectorySetpoint()
        message.timestamp = self.now_us()
        message.position = list(self.target)
        message.velocity = [nan, nan, nan]
        message.acceleration = [nan, nan, nan]
        message.jerk = [nan, nan, nan]
        message.yaw = self.target_yaw
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
                'PX4 reports a preflight warning; PX4 will enforce its arming '
                'checks when the command is sent.')
            self.preflight_warning_logged = True
        return telemetry_ready

    def target_reached(self) -> bool:
        north_error = self.position.x - self.target[0]
        east_error = self.position.y - self.target[1]
        horizontal_error = sqrt(north_error ** 2 + east_error ** 2)
        altitude_error = abs(self.position.z - self.target[2])
        return (
            horizontal_error <= self.ARRIVAL_RADIUS_M
            and altitude_error <= self.ALTITUDE_TOLERANCE_M
        )

    def start_flight(self) -> None:
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
        self.mission_started_ns = self.get_clock().now().nanoseconds
        self.get_logger().info('Armed in OFFBOARD; climbing to 10 m.')

    def begin_waypoint(self) -> None:
        name, north, east, down, yaw = self.WAYPOINTS[self.waypoint_index]
        self.target = (north, east, down)
        self.target_yaw = yaw
        self.hold_started_ns = None
        self.arrival_logged = False
        self.get_logger().info(
            f'Flying to inspection {self.waypoint_index + 1}/5: {name} '
            f'at NED [{north:.1f}, {east:.1f}, {down:.1f}].')

    def save_camera_frame(self, asset_name: str) -> tuple[str, str]:
        with self.camera_lock:
            frame = (
                None
                if self.latest_camera_frame is None
                else self.latest_camera_frame.copy()
            )
        if frame is None:
            return 'NO_FRAME', ''

        safe_name = ''.join(
            character.lower() if character.isalnum() else '_'
            for character in asset_name
        ).strip('_')
        filename = f'{self.waypoint_index + 1:02d}_{safe_name}.png'
        destination = self.image_directory / filename
        if cv2.imwrite(str(destination), frame):
            return 'CAPTURED', f'images/{filename}'
        return 'WRITE_FAILED', ''

    def log_inspection(self) -> None:
        name, north, east, down, _ = self.WAYPOINTS[self.waypoint_index]
        inspection_time = datetime.now().isoformat(timespec='seconds')
        camera_status, image_file = self.save_camera_frame(name)
        measured_north = round(float(self.position.x), 3)
        measured_east = round(float(self.position.y), 3)
        measured_down = round(float(self.position.z), 3)
        with self.csv_path.open('a', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow([
                inspection_time,
                name,
                north,
                east,
                down,
                measured_north,
                measured_east,
                measured_down,
                camera_status,
                image_file,
            ])
        self.inspection_records.append({
            'time': inspection_time,
            'asset': name,
            'target': (north, east, down),
            'measured': (measured_north, measured_east, measured_down),
            'camera_status': camera_status,
            'image_file': image_file,
        })
        self.get_logger().info(
            f'Inspection recorded: {name}; camera={camera_status}; '
            'holding for 3 seconds.')

    def write_html_report(self) -> None:
        if self.report_written:
            return
        rows = []
        for index, record in enumerate(self.inspection_records, start=1):
            target = record['target']
            measured = record['measured']
            if record['image_file']:
                image_html = (
                    f'<img src="{escape(record["image_file"])}" '
                    f'alt="{escape(record["asset"])}">')
            else:
                image_html = '<span class="missing">No frame captured</span>'
            rows.append(f'''<article class="inspection">
<div class="number">{index:02d}</div>
<div class="details"><h2>{escape(record['asset'])}</h2>
<p><strong>Time:</strong> {escape(record['time'])}</p>
<p><strong>Target NED:</strong> {target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f} m</p>
<p><strong>Measured NED:</strong> {measured[0]:.3f}, {measured[1]:.3f}, {measured[2]:.3f} m</p>
<p><strong>Camera:</strong> {escape(record['camera_status'])}</p></div>
<div class="evidence">{image_html}</div></article>''')

        generated = datetime.now().isoformat(timespec='seconds')
        html = f'''<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>UAV Substation Inspection Report</title>
<style>
body{{margin:0;background:#eef2f5;color:#17222b;font:16px Arial,sans-serif}}
header{{background:#102d3b;color:white;padding:32px 6vw;border-bottom:6px solid #f3b51b}}
main{{max-width:1100px;margin:28px auto;padding:0 20px}}
.summary{{background:white;padding:20px;border-radius:10px;margin-bottom:20px}}
.inspection{{display:grid;grid-template-columns:60px 1fr 360px;gap:18px;background:white;
margin:18px 0;padding:18px;border-radius:10px;box-shadow:0 2px 10px #0001}}
.number{{font-size:30px;font-weight:bold;color:#d49300}} h2{{margin-top:0}}
.evidence img{{width:100%;height:240px;object-fit:cover;border-radius:7px;background:#222}}
.missing{{display:grid;height:240px;place-items:center;background:#e3e7ea;border-radius:7px}}
@media(max-width:800px){{.inspection{{grid-template-columns:45px 1fr}}.evidence{{grid-column:1/-1}}}}
</style></head><body>
<header><h1>Autonomous UAV Substation Inspection</h1>
<p>ROS 2 Jazzy · PX4 SITL · Gazebo · Camera evidence</p></header>
<main><section class="summary"><h2>Mission summary</h2>
<p><strong>Result:</strong> Completed — returned, landed and disarmed</p>
<p><strong>Assets inspected:</strong> {len(self.inspection_records)} / {len(self.WAYPOINTS)}</p>
<p><strong>Generated:</strong> {escape(generated)}</p>
<p>Camera status reports frame capture only; no defect classifier is claimed.</p></section>
{''.join(rows)}</main></body></html>'''
        self.report_path.write_text(html, encoding='utf-8')
        self.report_written = True
        self.get_logger().info(f'HTML report written: {self.report_path}')

    def request_native_landing(self, reason: str) -> None:
        if self.state != MissionState.LANDING or not self.native_landing:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.native_landing = True
            self.state = MissionState.LANDING
            self.get_logger().info(reason)

    def begin_pad_landing(self) -> None:
        self.native_landing = False
        self.target = (0.0, 0.0, -1.5)
        self.state = MissionState.LANDING
        self.get_logger().info(
            'Home reached; holding the pad during descent to 1.5 m.')

    def mission_timed_out(self) -> bool:
        if self.mission_started_ns is None:
            return False
        elapsed = (
            self.get_clock().now().nanoseconds - self.mission_started_ns
        ) / 1_000_000_000
        return elapsed >= self.MAX_MISSION_SECONDS

    def log_waiting_periodically(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_wait_log_ns >= 5_000_000_000:
            self.get_logger().info('Waiting for PX4 SITL and ROS 2 telemetry...')
            self.last_wait_log_ns = now_ns

    def timer_callback(self) -> None:
        if self.state == MissionState.COMPLETE:
            return

        if self.state == MissionState.WAITING_FOR_PX4:
            if not self.px4_is_ready():
                self.log_waiting_periodically()
                return
            self.state = MissionState.STREAMING_SETPOINTS
            self.get_logger().info('PX4 ready; warming up setpoint stream.')

        if self.state in (
            MissionState.STREAMING_SETPOINTS,
            MissionState.TAKING_OFF,
            MissionState.INSPECTING,
            MissionState.RETURNING,
            MissionState.LANDING,
        ):
            if self.state != MissionState.LANDING or not self.native_landing:
                self.publish_offboard_heartbeat()
                self.publish_target()

        if self.state == MissionState.STREAMING_SETPOINTS:
            self.warmup_cycles += 1
            if self.warmup_cycles >= self.SETPOINT_WARMUP_CYCLES:
                self.start_flight()
            return

        if self.status.failsafe:
            self.request_native_landing(
                'PX4 failsafe detected; native landing requested.')
            return

        if self.mission_timed_out():
            self.request_native_landing(
                'Mission timeout reached; native landing requested.')
            return

        if self.state == MissionState.TAKING_OFF:
            if self.target_reached():
                self.state = MissionState.INSPECTING
                self.get_logger().info('Cruise altitude reached.')
                self.begin_waypoint()
            return

        if self.state == MissionState.INSPECTING:
            if not self.target_reached():
                self.hold_started_ns = None
                return

            if not self.arrival_logged:
                self.log_inspection()
                self.arrival_logged = True
                self.hold_started_ns = self.get_clock().now().nanoseconds
                return

            held_seconds = (
                self.get_clock().now().nanoseconds - self.hold_started_ns
            ) / 1_000_000_000
            if held_seconds < self.INSPECTION_HOLD_SECONDS:
                return

            self.waypoint_index += 1
            if self.waypoint_index < len(self.WAYPOINTS):
                self.begin_waypoint()
            else:
                self.state = MissionState.RETURNING
                self.target = (0.0, 0.0, self.CRUISE_ALTITUDE_NED)
                self.get_logger().info(
                    'All five assets inspected; returning to the landing pad.')
            return

        if self.state == MissionState.RETURNING:
            if self.target_reached():
                self.begin_pad_landing()
            return

        if self.state == MissionState.LANDING:
            if not self.native_landing and self.target_reached():
                self.request_native_landing(
                    'Precision approach complete; PX4 final touchdown requested.')
                return
            if (
                not self.native_landing
                and self.land_detected is not None
                and self.land_detected.landed
                and not self.disarm_sent
            ):
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=0.0,
                )
                self.disarm_sent = True
                self.get_logger().info(
                    'Landing detector confirms touchdown; disarm requested.')
            if self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.state = MissionState.COMPLETE
                self.write_html_report()
                self.get_logger().info(
                    'Inspection mission complete: landed and disarmed safely.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointInspection()
    try:
        while rclpy.ok() and node.state != MissionState.COMPLETE:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.get_logger().info('Inspection mission stopped by operator.')
    finally:
        node.gz_node.unsubscribe(node.CAMERA_TOPIC)
        node.gz_node = None
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
