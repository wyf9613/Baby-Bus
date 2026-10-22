#!/usr/bin/env bash
# Record the MPC/lattice car-run topics in one command (run on the car, ROS sourced).
#
#     ~/record_car_bag.sh carrun-cl-nolimit-blueleft
#
# Writes ~/ai4r-car-bags/<name>-<YYYYmmdd-HHMMSS>; Ctrl+C stops. Name defaults to "run".
set -euo pipefail
name="${1:-run}"
out="$HOME/ai4r-car-bags/${name}-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$HOME/ai4r-car-bags"
echo "Recording to $out"
exec ros2 bag record -o "$out" --topics \
  /car/mpc_debug /car/mpc_prediction /car/policy_trajectory /car/planner_execution \
  /car/shadow_trajectory /car/shadow_checked_prefix /car/shadow_reference /car/shadow_planning \
  /car/policy_fsm_state_string /car/traxxas_state \
  /car/drive_and_steer_set_point_normalized /car/drive_and_steer_applied_normalized \
  /car/wheel_speed_m_per_sec /car/cone_detections /car/imu/data /car/diagnostics /tf /tf_static
