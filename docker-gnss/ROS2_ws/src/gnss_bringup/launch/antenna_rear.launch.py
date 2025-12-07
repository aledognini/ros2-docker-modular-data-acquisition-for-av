import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, Node


def generate_launch_description():
  launch_desc = LaunchDescription()
  config_rear = os.path.join(get_package_share_directory('gnss_bringup'),
                       'config',
                       'rear_antenna.yaml'
                       )
  node = Node(
      package='swiftnav_ros2_driver',
      executable='sbp-to-ros',
      parameters=[config_rear],
      remappings=[
            ('gpsfix', 'antenna_rear/gpsfix'),
            ('navsatfix', 'antenna_rear/navsatfix'),
            ('twistwithcovariancestamped', 'antenna_rear/twistwithcovariancestamped'),
            ('baseline', 'antenna_rear/baseline'),
            ('timereference', 'antenna_rear/timereference'),
            ('imu', 'antenna_rear/imu')
      ]
  )
  launch_desc.add_action(node)
  return launch_desc
