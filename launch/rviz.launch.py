import launch

from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = launch.LaunchDescription()

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))

    # #{ refactor Rviz script

    refactor_rviz_config_cmd = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([
                FindPackageShare('vins_mah'),
                'scripts',
                'refactor_rviz_config.sh'
            ])
        ],
        output='screen'
    )

    ld.add_action(refactor_rviz_config_cmd)

    # #}

    # #{ rviz node

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-d /tmp/default.rviz'],
        prefix=["bash -c 'sleep 2; $0 $@'"])

    ld.add_action(rviz_node)

    # #}

    return ld
