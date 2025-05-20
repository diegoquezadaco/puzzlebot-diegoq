from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    package_directory = get_package_share_directory('mobile_robotics')

    #     # Load the parameters from the YAML file
    # config = os.path.join(
    #     package_directory,
    #     'config',
    #     'path_params.yaml'
    # )
    
    configcv = os.path.join(
        package_directory,
        'config',
        'cv_params.yaml'
    )

    color_tracker = Node(
        package='mobile_robotics',
        executable='color_tracker',
        name='color_tracker',
        parameters=[{'use_sim_time': True}]
    )

    line_follower_control = Node(
        package='mobile_robotics',
        executable='line_follower_control',
        name='line_follower_control',
        parameters=[{'use_sim_time': True}]
    )
    
    cv_decision_making = Node(
        package='mobile_robotics',
        executable='cv_decision_making',
        name='cv_decision_making',
        parameters=[{'use_sim_time': False}, configcv]
    )

    


    return LaunchDescription([
        color_tracker,
        line_follower_control,
        cv_decision_making,
    ])