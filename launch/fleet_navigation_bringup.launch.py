import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def load_config(robot_id):
    ns = f'fleet_{robot_id}'
    config_file = os.path.join(
        os.path.dirname(__file__), '..', 'config', f'{ns}_config.yaml'
    )
    config_file = os.path.abspath(config_file)
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    return (
        config.get('pc_topic'),
        config.get('imu_in'),
        config.get('imu_out'),
        config.get('security_distance'),
        config.get('max_slope'),
        config.get('ground_clearance'),
        config.get('robot_height'),
        config.get('robot_width'),
        config.get('robot_length')
    )




def generate_launch_description():

    ld = LaunchDescription()


    # ------------------ sim time ------------------
    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true.'
    )
    ld.add_action(declare_sim_time_arg)
    use_sim_time = LaunchConfiguration('use_sim_time')



    # ------------------ Multi-robot loop ------------------
    fleet_bringup_dir = get_package_share_directory('fleet_navigation_bringup')                

    NUM_ROBOTS = 2
    for robot_id in range(NUM_ROBOTS):

        # create robot name and param file path
        robot_name = f'fleet_{robot_id}'                    
        param_file_name = f"{robot_name}_nav2_params.yaml"
        param_file_path = os.path.join(fleet_bringup_dir, 'params', param_file_name)
        ld.add_action(LogInfo(msg=f"[DEBUG] Robot {robot_name} params: {param_file_path}"))


        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('fleet_navigation_bringup'),
                    'launch',
                    'custom_nav2_bringup.launch.py'
                )
            ),
            launch_arguments={                
                'use_sim_time': use_sim_time,
                'params_file': param_file_path,
                'use_namespace': 'True',
                'namespace': robot_name,
                'use_composition': 'True',
            }.items()
        )

        # Load traversability map config
        (pc_topic, imu_in, imu_out, security_distance, max_slope, ground_clearance, robot_height, robot_width, robot_length) = load_config(robot_id)

        traversability_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('traversability_map'),
                    'launch',
                    'traversabilityMap_launch.py'
                )
            ),
            launch_arguments={
                'ns': robot_name,  # Namespace for traversability node
                'pc_topic': pc_topic,
                'imu_in': imu_in,
                'imu_out': imu_out,
                'security_distance': str(security_distance),
                'max_slope': str(max_slope),
                'ground_clearance': str(ground_clearance),
                'robot_height': str(robot_height),
                'robot_width': str(robot_width),
                'robot_length': str(robot_length)
            }.items()
        )

        goto_remaps = Node(
            package='fleet_navigation_bringup',
            executable='goto_remaps_node',
            name=f'goto_remaps_fleet_{robot_id}',
            #namespace=f'fleet_{robot_id}',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_id': robot_id,
            }]
        )

        robot_group = GroupAction([
            #PushRosNamespace(namespace = robot_name),
            nav2_launch,
            traversability_launch,
            goto_remaps
        ])

        # Add to launch description
        ld.add_action(robot_group)        

    return ld



# import os
# import yaml

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node



# def load_config(robot_id):
#     ns = f'fleet_{robot_id}'
#     config_file = os.path.join(
#         os.path.dirname(__file__), '..', 'config', f'{ns}_config.yaml'
#     )
#     config_file = os.path.abspath(config_file)
#     with open(config_file, 'r') as f:
#         config = yaml.safe_load(f)
#     return config.get('pc_topic'), config.get('imu_in'), config.get('imu_out'), config.get('security_distance'), config.get('max_slope'), config.get('ground_clearance'), config.get('robot_height'), config.get('robot_width'), config.get('robot_length')




# def generate_launch_description():

#     # ───── Set up paths and environment ────────────────────────────────────────
#     fleet_bringup_dir = get_package_share_directory('fleet_navigation_bringup')                

#     declare_sim_time_arg = DeclareLaunchArgument(
#         'use_sim_time',
#         default_value='true',
#         description='Use simulation (Gazebo) clock if true.'
#     )

#     # Create main launch description
#     ld = LaunchDescription()
#     ld.add_action(declare_sim_time_arg)

#     # ───── For each robot ────────────────────────────────────────
#     for robot_id in range(2):
#         robot_name = f'fleet_{robot_id}'
#         param_file_name = f"{robot_name}_params.yaml"
#         param_file_path = os.path.join(fleet_bringup_dir, 'params', param_file_name)

#         # ───── Launch arguments ────────────────────────────────────────────────────        
#         use_sim_time = LaunchConfiguration('use_sim_time')

#         ld.add_action(LogInfo(msg=f"[DEBUG] Robot {robot_name} params: {param_file_path}"))


#         nav2_launch = IncludeLaunchDescription(

#             PythonLaunchDescriptionSource(
#                 os.path.join(
#                     get_package_share_directory('fleet_navigation_bringup'),
#                     'launch',
#                     'custom_navigation_launch.py'
#                 )
#             ),
#             launch_arguments={
#                 'namespace': robot_name,
#                 'use_sim_time': use_sim_time,
#                 'autostart': 'true',
#                 'params_file': param_file_path,
#                 'use_respawn': 'False',
#                 'log_level': 'info'
#             }.items()
#         )


#         pc_topic, imu_in, imu_out, security_distance, max_slope, ground_clearance, robot_height, robot_width, robot_length = load_config(robot_id)

#         traversability_launch = IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join(
#                     get_package_share_directory('traversability_map'),
#                     'launch',
#                     'traversabilityMap_launch.py'
#                 )
#             ),
#             launch_arguments={
#                 'ns': robot_name,
#                 'pc_topic': pc_topic,
#                 'imu_in': imu_in,
#                 'imu_out': imu_out,
#                 'security_distance': str(security_distance),
#                 'max_slope': str(max_slope),
#                 'ground_clearance': str(ground_clearance),
#                 'robot_height': str(robot_height),
#                 'robot_width': str(robot_width),
#                 'robot_length': str(robot_length)
#             }.items()
#         )

#         ld.add_action(nav2_launch)
#         ld.add_action(traversability_launch)

#     return ld