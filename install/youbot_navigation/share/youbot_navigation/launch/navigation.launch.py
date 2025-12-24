from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    #this is a navigation2 stack launch file
    #we have already made the description package and that one loads the urdf file of the youbot with mecanum wheel controller
    #now here we are just going to call that lauch file and then after that we are going to lauch other stuff navigation stuff as well

    #calling out the description package

    description_pkg = get_package_share_directory('youbot_description')
    nav_pkg = get_package_share_directory('youbot_navigation')
    slam_config = os.path.join(nav_pkg,'config','mapper_params_online_async.yaml')


    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource
        (
            [os.path.join(description_pkg,'launch','gazebo.launch.py')]
        )
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(nav_pkg,'launch','slam.launch.py')]
        )
    )
    




   
    return LaunchDescription([
        description,
    ])
