"""Check real ROS imports and synthetic LaserScan -> MCAP -> LaserScan IO.

Run inside the prepared ROS shell. No ROS node, driver or vehicle is started.
The scan below is synthetic setup data, not a recording from the car.
"""
import argparse
from importlib.metadata import version
import json
import os
from pathlib import Path
import platform
import tempfile

from ament_index_python.packages import get_package_prefix
from dream_interfaces.msg import ConeDetection, DriveAndSteer, FiducialDetections
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py
from sensor_msgs.msg import LaserScan


def check():
    topic = '/babybus/setup_scan'
    stamps = [1_000_000_000, 1_100_000_000]
    with tempfile.TemporaryDirectory(prefix='babybus-ros-probe-') as temporary:
        bag = str(Path(temporary)/'synthetic_scan')
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
                    rosbag2_py.ConverterOptions('', ''))
        writer.create_topic(rosbag2_py.TopicMetadata(id=0, name=topic,
            type='sensor_msgs/msg/LaserScan', serialization_format='cdr'))
        for index, stamp in enumerate(stamps):
            scan = LaserScan()
            scan.header.frame_id = 'synthetic_laser'
            scan.header.stamp.sec, scan.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            scan.angle_min, scan.angle_max, scan.angle_increment = -.1, .1, .1
            scan.range_min, scan.range_max = .15, 16.
            scan.ranges = [.5, .4-index*.01, .5]
            writer.write(topic, serialize_message(scan), stamp)
        del writer
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
                    rosbag2_py.ConverterOptions('', ''))
        records = []
        while reader.has_next():
            name, data, stamp = reader.read_next()
            scan = deserialize_message(data, LaserScan)
            assert name == topic and scan.header.frame_id == 'synthetic_laser'
            assert scan.header.stamp.sec*1_000_000_000+scan.header.stamp.nanosec == stamp
            assert abs(scan.ranges[1]-(.4-len(records)*.01)) < 1e-6
            records.append(stamp)
        del reader
        assert records == stamps
    # Construct each shared type to prove generated ROS interfaces are usable.
    ConeDetection(); DriveAndSteer(); FiducialDetections()
    return dict(ros_distro=os.environ['ROS_DISTRO'], architecture=platform.machine(),
        python=platform.python_version(), rclpy=version('rclpy'),
        interfaces_prefix=get_package_prefix('dream_interfaces'),
        policy_prefix=get_package_prefix('ai4r_policy'),
        synthetic_mcap_roundtrip_messages=len(records), timestamps_preserved=True,
        ranges_preserved=True, physical_sensor_used=False, vehicle_used=False, passed=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    result = check()
    encoded = json.dumps(result, ensure_ascii=False, indent=2)+'\n'
    if args.report:
        args.report.write_text(encoded)
    print(encoded)
