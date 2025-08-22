# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """launch内容描述函数,由ros2 launch 扫描调用"""
    position_node = Node(
        package="get_position",
        executable="pub_position"
    )
    heart_node = Node(
        package="get_position",
        executable="pub_heart"
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [position_node, heart_node])
    # 返回让ROS2根据launch描述执行节点
    return launch_description
