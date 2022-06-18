from launch import LaunchDescription
from launch.actions import  ExecuteProcess


def generate_launch_description():
 
  return LaunchDescription([
   
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '/home/cc/pg-ros/6_ROS2_Robotics/prius/ws/src/world/prius_on_track.world'],
            output='screen'),
  
  ])
