from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ## ***** Launch arguments *****
    # 是否使用仿真时间，真实的机器人我们不需要，设置为False
    use_sim_time_arg = LaunchConfiguration('use_sim_time', default='False')
    rviz2_config = os.path.join(get_package_share_directory('limo_bringup'),'rviz','cartographer.rviz')
    ## ***** File paths ******
    # 找到cartographer功能包的地址
    pkg_share = FindPackageShare('cartographer_ros').find('cartographer_ros')
    # 地图的分辨率
    resolution = LaunchConfiguration('resolution', default='0.05')
    # 地图的发布周期
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': use_sim_time_arg}],
        arguments = [
            '-configuration_directory', FindPackageShare('limo_bringup').find('limo_bringup') + '/config_files',
            '-configuration_basename', 'limo_lds_2d.lua'],
        remappings = [
            ('echoes', 'horizontal_laser_2d')],
        output = 'screen'
        )

	# 可视化节点
    rviz_node = Node(
          package='rviz2',
          namespace='rviz2',
          executable='rviz2',
          name='rviz2',
          arguments=['-d',rviz2_config],
          parameters=[{'use_sim_time': use_sim_time_arg}],
          output='screen')

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [{'use_sim_time': use_sim_time_arg}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    return LaunchDescription([
        # Nodes
        rviz_node ,
        cartographer_node,
        cartographer_occupancy_grid_node,
       # laser_to_map_tf_node,
    ])


