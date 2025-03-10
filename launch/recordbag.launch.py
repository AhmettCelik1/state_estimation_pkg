from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    
    mahony_ahrs_pose = Node(
            package='state_estimation_pkg',
            executable='quat2euler',
            name='quat2euler_mahony_ahrs',
             parameters=[{'topic_name': '/imu/mahony_ahrs/pose'}],
        )
    
    imu_tools_com_non_adaptive_pose = Node(
            package='state_estimation_pkg',
            executable='quat2euler',
            name='quat2euler_imu_tools_com_non_adaptive_pose',
             parameters=[{'topic_name': '/imu/tools/com_non_adaptive/pose'}],
        )
    
    
    imu_tools_com_adaptive_pose = Node(
            package='state_estimation_pkg',
            executable='quat2euler',
            name='quat2euler_imu_tools_com_adaptive_pose',
             parameters=[{'topic_name': '/imu/tools/com_adaptive/pose'}],
        )
    
    imu_tools_madwick_pose = Node(
            package='state_estimation_pkg',
            executable='quat2euler',
            name='quat2euler_imu_tools_madwick_pose',
             parameters=[{'topic_name': '/imu/tools/madwick/pose'}],
        )
    
    imu_ground_truth = Node(
            package='state_estimation_pkg',
            executable='quat2euler',
            name='quat2euler_imu_ground_truth',
             parameters=[{'topic_name': '/imu/ground_truth'}],
        )
     
    
    publishimu = Node(
            package='state_estimation_pkg',
            executable='publishimu',
            name='publishimu'
        )

        
    return LaunchDescription(
            [
           
                mahony_ahrs_pose,
                imu_tools_com_non_adaptive_pose,
                imu_tools_com_adaptive_pose,
                imu_tools_madwick_pose,
                imu_ground_truth,
                # publishimu
            ]
        )
