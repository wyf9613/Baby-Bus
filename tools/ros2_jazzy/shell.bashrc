# Loaded explicitly by the local developer-shell helper.
source /opt/ros/jazzy/setup.bash
if [[ -n "${AI4R_POLICY_SOURCE:-}" && -f "$AI4R_POLICY_SOURCE/.verification/ws/install/setup.bash" ]]; then
  source "$AI4R_POLICY_SOURCE/.verification/ws/install/setup.bash"
fi
export ROS_DOMAIN_ID=218
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PS1='(Baby-Bus ROS Jazzy) \w\$ '
