from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4","--port","8888"],
            output="log"
        ),
        Node(
            package="imav24",
            executable="px4_driver",
            output="screen"
        ),
        Node(
            package="aruco_opencv",
            executable="aruco_tracker_autostart",
            output="screen",
            parameters=[{
                "cam_base_topic":"/pi_camera/image_raw",
                "marker_dict":"5X5_1000"
            }]
        )
    ])