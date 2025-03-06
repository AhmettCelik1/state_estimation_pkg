from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    comp_filter_node = Node(
            package='state_estimation_pkg',
            executable='comp_filter',
            name='quaternion_complementary_filter'
        )
    mahony_ahrs_node = Node(
            package='state_estimation_pkg',
            executable='mahony_ahrs',
            name='mahonyAHRS'
        )
    
    publishattitude = Node(
            package='state_estimation_pkg',
            executable='publishattitude',
            name='publishattitude'
        )
    

        
    return LaunchDescription(
            [
                comp_filter_node,
                mahony_ahrs_node,
                publishattitude,
            ]
        )
