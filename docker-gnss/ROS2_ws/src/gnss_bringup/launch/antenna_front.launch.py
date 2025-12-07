import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, Node


def generate_launch_description():
  launch_desc = LaunchDescription()
  config_front = os.path.join(get_package_share_directory('gnss_bringup'),
                        'config',
                        'front_antenna.yaml'
                        )
  node = Node(
      package='swiftnav_ros2_driver',
      executable='sbp-to-ros',
      parameters=[config_front],
      remappings=[
            ('gpsfix', 'antenna_front/gpsfix'),
            ('navsatfix', 'antenna_front/navsatfix'),
            ('twistwithcovariancestamped', 'antenna_front/twistwithcovariancestamped'),
            ('baseline', 'antenna_front/baseline'),
            ('timereference', 'antenna_front/timereference'),
            ('imu', 'antenna_front/imu')
      ]
  )
  launch_desc.add_action(node)
  return launch_desc
