import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  # Get the path to the RViz config file
  rviz_config_file = os.path.join(
      get_package_share_directory('pedestrian_visualizer'),
      'rviz',
      'pedestrian.rviz')

  # Node for the pedestrian warning visualizer
  visualizer_node = Node(
      package='pedestrian_visualizer',
      executable='pedestrian_warning_node',
      name='pedestrian_warning_node'
  )

  # Node for RViz2
  rviz_node = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      arguments=['-d', rviz_config_file]
  )

  return LaunchDescription([
      visualizer_node,
      rviz_node
  ])
