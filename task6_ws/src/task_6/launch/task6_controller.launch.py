from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    ld = LaunchDescription()
    controller1_node = Node(
        package="task_6",           # Enter the name of your ROS2 package
        executable="controller_1",    # Enter the name of your executable
    )
    controller2_node = Node(
        package="task_6",           # Enter the name of your ROS2 package
        executable="controller_2",    # Enter the name of your executable
    )
    controller3_node = Node(
        package="task_6",           # Enter the name of your ROS2 package
        executable="controller_3",    # Enter the name of your executable
    )
    ld.add_action(controller1_node)
    ld.add_action(controller2_node)
    ld.add_action(controller3_node)

    return ld 