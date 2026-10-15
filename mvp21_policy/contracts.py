"""MVP2.1 contracts; relocated from pinned f11428b, without algorithm tuning."""

from dataclasses import dataclass


FSM_STATE_NOT_PUBLISHING_ACTIONS = 1


FSM_STATE_PUBLISHING_ZERO_ACTIONS = 2


FSM_STATE_PUBLISHING_POLICY_ACTION = 3


STATE_NAMES = {1: "Not publishing any actions", 2: "Publishing zero actions",
               3: "Publishing policy actions"}


SENSORS = ("cone_detections", "fiducial_detections", "lidar_scan", "lidar_cartesian", "wheel_speed", "imu_orientation",
           "imu_angular_velocity", "imu_specific_force")


STAMPED_SENSORS = tuple(name for name in SENSORS if name != "wheel_speed")


FIDUCIAL_DICTIONARY_SIZES = {
    **{f"DICT_{bits}X{bits}_{count}": count
       for bits in (4, 5, 6, 7) for count in (50, 100, 250, 1000)},
    "DICT_ARUCO_ORIGINAL": 1024,
}


@dataclass
class Observation:
    """One accepted measurement, with independent receipt and sample times."""
    value: object
    received_at: float             # monotonic seconds, not a ROS timestamp
    stamp_ns: int | None = None     # wheel-speed messages have no header
    received_ros_ns: int | None = None  # receipt proxy, NOT wheel acquisition time
