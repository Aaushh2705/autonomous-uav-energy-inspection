#!/usr/bin/env bash

M2_LOG_DIR=${MISSION_LOG_DIR:-/home/aayush1/uav_ws/log/milestone2_test}
M2_MISSION_EXECUTABLE=${MISSION_EXECUTABLE:-offboard_mission}
M2_MISSION_TIMEOUT=${MISSION_TIMEOUT:-40}
M2_SUCCESS_TEXT=${MISSION_SUCCESS_TEXT:-Mission complete: vehicle landed and disarmed safely.}
M2_RESULT_LABEL=${MISSION_RESULT_LABEL:-MILESTONE2_RESULT}
M2_VEHICLE_MODEL=${PX4_VEHICLE_MODEL:-gz_x500}
M2_MODEL_DIRECTORY=${PX4_MODEL_DIRECTORY:-/home/aayush1/PX4-Autopilot/Tools/simulation/gz/models}
mkdir -p "$M2_LOG_DIR"

source /opt/ros/jazzy/setup.bash
source /home/aayush1/uav_ws/install/setup.bash
source /home/aayush1/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh
set -u

M2_WORLD_FILE=/home/aayush1/uav_ws/src/uav_energy_inspection/worlds/energy_substation.sdf
export GZ_SIM_RESOURCE_PATH=/home/aayush1/uav_ws/src/uav_energy_inspection/models:/home/aayush1/uav_ws/src/uav_energy_inspection/worlds:$GZ_SIM_RESOURCE_PATH

cleanup_milestone2() {
    for m2_pid in "${M2_GCS_PID:-}" "${M2_PX4_PID:-}" "${M2_GZ_PID:-}" "${M2_AGENT_PID:-}"; do
        if [[ -n "$m2_pid" ]]; then
            kill -- "-$m2_pid" 2>/dev/null || kill "$m2_pid" 2>/dev/null || true
        fi
    done
}
trap cleanup_milestone2 EXIT INT TERM

setsid MicroXRCEAgent udp4 -p 8888 -v 1 \
    >"$M2_LOG_DIR/agent.log" 2>&1 &
M2_AGENT_PID=$!

setsid ros2 run uav_energy_inspection sitl_gcs_heartbeat \
    >"$M2_LOG_DIR/gcs_heartbeat.log" 2>&1 &
M2_GCS_PID=$!

setsid gz sim -r "$M2_WORLD_FILE" \
    >"$M2_LOG_DIR/gazebo.log" 2>&1 &
M2_GZ_PID=$!
sleep 5

cd /home/aayush1/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_bridge
setsid /usr/bin/cmake -E env PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 \
    PX4_SIM_MODEL="$M2_VEHICLE_MODEL" PX4_GZ_WORLD=energy_substation \
    PX4_GZ_MODELS="$M2_MODEL_DIRECTORY" \
    PX4_GZ_MODEL_POSE=-30,0,0.25,0,0,0 \
    PX4_GZ_FOLLOW_OFFSET_X=-14 PX4_GZ_FOLLOW_OFFSET_Y=-14 \
    PX4_GZ_FOLLOW_OFFSET_Z=9 GZ_IP=127.0.0.1 \
    /home/aayush1/PX4-Autopilot/build/px4_sitl_default/bin/px4 -d \
    >"$M2_LOG_DIR/px4.log" 2>&1 &
M2_PX4_PID=$!

echo "Waiting for PX4 ROS 2 telemetry..."
M2_TOPICS_READY=0
for _ in $(seq 1 50); do
    if ros2 topic list 2>/dev/null | grep -q '/fmu/out/vehicle_status_v4'; then
        M2_TOPICS_READY=1
        break
    fi
    sleep 0.5
done

if [[ "$M2_TOPICS_READY" -ne 1 ]]; then
    echo "PX4 telemetry did not become ready."
    exit 1
fi

sleep 6
echo "Running ${M2_MISSION_EXECUTABLE}..."
timeout --signal=INT "$M2_MISSION_TIMEOUT" \
    ros2 run uav_energy_inspection "$M2_MISSION_EXECUTABLE" \
    2>&1 | tee "$M2_LOG_DIR/mission.log"

echo "Final PX4 status:"
timeout 8 ros2 topic echo --once /fmu/out/vehicle_status_v4 \
    | grep -E 'arming_state:|nav_state:|failsafe:|pre_flight_checks_pass:' || true

echo "Final local position:"
timeout 8 ros2 topic echo --once /fmu/out/vehicle_local_position_v1 \
    | grep -E '^x:|^y:|^z:' || true

if grep -q "$M2_SUCCESS_TEXT" \
    "$M2_LOG_DIR/mission.log"; then
    echo "${M2_RESULT_LABEL}=PASS"
    if [[ "${KEEP_GAZEBO_OPEN:-0}" == "1" ]]; then
        echo "Gazebo will remain open. Press Ctrl+C here when you finish exploring."
        while true; do sleep 1; done
    fi
else
    echo "${M2_RESULT_LABEL}=FAIL"
    exit 1
fi
