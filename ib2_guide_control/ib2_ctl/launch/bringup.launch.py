import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

  return LaunchDescription([
    # Declare the launch argument
    DeclareLaunchArgument(
      'use_fsm',
      default_value='true',
      description='Whether to use the fsm package or not'
    ),
    OpaqueFunction(function=launch_setup)
  ])

def launch_setup(context, *args, **kwargs):
  # Get the launch arguments
  use_fsm = LaunchConfiguration('use_fsm').perform(context)

  # Get the share directory of the ctl_only and prop packages
  ctl_only_share_dir = get_package_share_directory('ib2_ctl')
  fsm_share_dir = get_package_share_directory('ib2_fsm')
  prop_share_dir = get_package_share_directory('ib2_prop')

  # Construct the full path to the YAML configuration files
  ctl_yaml = os.path.join(ctl_only_share_dir, 'config', 'ctl.yaml')
  prop_yaml = os.path.join(prop_share_dir, 'config', 'prop.yaml')

  nodes_to_launch = []

  nodes_to_launch.extend([
    Node(
      package='ib2_ctl',
      executable='ib2_ctl_node',
      name='ctl_only',
      parameters=[ctl_yaml,
        {
          'use_sim_time': True
        }
      ]
    ),
    # Node(
    #   package='ib2_prop',
    #   executable='ib2_prop_node',
    #   name='prop',
    #   parameters=[prop_yaml]
    # ),
  ])

  # # If use_fsm is true, add the fsm node
  # if use_fsm.lower() == 'true':
  #   nodes_to_launch.extend([
  #     Node(
  #       package='ib2_fsm',
  #       executable='ib2_fsm_node',
  #       name='fsm',
  #       parameters=[ctl_yaml]
  #     ),
  #   ])

  return nodes_to_launch
