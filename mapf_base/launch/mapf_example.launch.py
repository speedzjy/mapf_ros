from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode, ParameterFile


def generate_launch_description():
    # Declare map file argument
    map_file_arg = DeclareLaunchArgument(
        name="map",
        default_value="warehouse_low_reso_1.5.yaml",
        description="YAML map file name",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    autostart_arg = DeclareLaunchArgument(
        name="autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    map_file = PathJoinSubstitution(
        [
            FindPackageShare("multi_ridgeback_sim"),
            "maps",
            LaunchConfiguration("map"),
        ]
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    lifecycle_nodes = ["map_server", "mapf_base_node"]

    mapf_params = PathJoinSubstitution(
        [
            FindPackageShare("mapf_base"),
            "params",
            "mapf_params.yaml",
        ]
    )
    costmap_params = PathJoinSubstitution(
        [
            FindPackageShare("mapf_base"),
            "params",
            "costmap_params.yaml",
        ]
    )

    return LaunchDescription(
        [
            map_file_arg,
            use_sim_time_arg,
            autostart_arg,
            GroupAction(
                [
                    Node(
                        namespace="mapf",
                        package="nav2_map_server",
                        executable="map_server",
                        name="map_server",
                        output="screen",
                        respawn=True,
                        parameters=[
                            mapf_params,
                            {"yaml_filename": map_file},
                        ],
                    ),
                    Node(
                        namespace="mapf",
                        package="mapf_base",
                        executable="mapf_base_node",
                        name="mapf_base_node",
                        output="screen",
                        respawn=True,
                        parameters=[
                            costmap_params,
                            mapf_params,
                            {"mapf_planner": "mapf_planner/ECBSROS"},
                        ],
                    ),
                    Node(
                        namespace="mapf",
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_mapf",
                        output="screen",
                        parameters=[
                            {"use_sim_time": use_sim_time},
                            {"autostart": autostart},
                            {"node_names": lifecycle_nodes},
                        ],
                    ),
                ]
            ),
            # 3. Launch goal_transformer and plan_executor
            TimerAction(
                period=2.0,
                actions=[
                    GroupAction(
                        [
                            Node(
                                namespace="mapf",
                                package="mapf_base",
                                executable="goal_transformer",
                                name="goal_transformer",
                                output="screen",
                                parameters=[mapf_params],
                            ),
                            Node(
                                namespace="mapf",
                                package="mapf_base",
                                executable="plan_executor",
                                name="plan_executor",
                                output="screen",
                                arguments=["--ros-args", "--log-level", "info"],
                                parameters=[mapf_params],
                            ),
                        ]
                    )
                ],
            ),
        ]
    )
