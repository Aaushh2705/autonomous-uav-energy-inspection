#!/usr/bin/env bash

export MISSION_EXECUTABLE=waypoint_inspection
export MISSION_TIMEOUT=300
export MISSION_LOG_DIR=/home/aayush1/uav_ws/log/milestone3_test
export MISSION_SUCCESS_TEXT='Inspection mission complete: landed and disarmed safely.'
export MISSION_RESULT_LABEL=MILESTONE3_RESULT
export PX4_VEHICLE_MODEL=gz_inspection_x500
export PX4_MODEL_DIRECTORY=/home/aayush1/uav_ws/src/uav_energy_inspection/models

exec /home/aayush1/uav_ws/src/uav_energy_inspection/scripts/run_milestone2_test.sh
