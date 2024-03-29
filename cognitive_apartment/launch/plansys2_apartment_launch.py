# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('cognitive_apartment')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/cognitive_apartment.pddl',
          'namespace': namespace
        }.items())

    nav2_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('robots'),
        'launch',
        'tiago_nodoors.launch.py')),
    launch_arguments={
        'autostart': 'true'
    }.items())

    # Specify the actions
    blackboard = Node(
        package='blackboard',
        executable='blackboard_main',
        name='blackboard_main',
        namespace=namespace,
        output='screen')
        #parameters=[example_dir + '/config/params.yaml'])
        
    sync = Node(
        package='cognitive_apartment',
        executable='sync_node',
        name='sync_node',
        namespace=namespace,
        output='screen',
        parameters=[example_dir + '/config/params.yaml'])
        
    move = Node(
        package='cognitive_apartment',
        executable='move_action_node',
        name='move_action_node',
        namespace=namespace,
        output='screen')
        #parameters=[example_dir + '/config/params.yaml'])
    
    explore = Node(
        package='cognitive_apartment',
        executable='explore_action_node',
        name='explore_action_node',
        namespace=namespace,
        output='screen')
        #parameters=[example_dir + '/config/params.yaml'])
    
    grab_object = Node(
        package='cognitive_apartment',
        executable='grab_object_action_node',
        name='grab_object_action_node',
        namespace=namespace,
        output='screen')
        #parameters=[example_dir + '/config/params.yaml'])
    
    release_object = Node(
        package='cognitive_apartment',
        executable='release_object_action_node',
        name='release_object_action_node',
        namespace=namespace,
        output='screen')
        #parameters=[example_dir + '/config/params.yaml'])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)
    ld.add_action(nav2_cmd)

    ld.add_action(blackboard)
    ld.add_action(sync)
    ld.add_action(move)
    ld.add_action(explore)
    ld.add_action(grab_object)
    ld.add_action(release_object)

    return ld