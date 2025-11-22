from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable, TimerAction  #edited


def generate_launch_description():
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Set to false to launch Gazebo without GUI'
    )

    package_arg = DeclareLaunchArgument('urdf_package',
                                        description='The package where the robot description is located',
                                        default_value='simulation')

    model_arg = DeclareLaunchArgument('urdf_package_path',
                                      description='The path to the robot description relative to the package root',
                                      default_value='urdf/my_rover.urdf.xacro')

    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=PathJoinSubstitution([FindPackageShare('simulation'), 'worlds', 'default.sdf']),
        description='Path to the world file to load'
    )

    pkg_simulation = FindPackageShare('simulation')

    server_config_path = PathJoinSubstitution([
      pkg_simulation, 'config', 'server.config'
    ])

    bridge_config_path = PathJoinSubstitution([
      pkg_simulation, 'config', 'bridge.yaml'
    ])

    ##edited 
    gz_resource_path = [PathJoinSubstitution([FindPackageShare('simulation'), 'models'])]
    gz_resource_env = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path)


    gz_args = [TextSubstitution(text='-r -v 1 '), LaunchConfiguration('world')]

    empty_world_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'pause': 'true',  
            'server_config_file': server_config_path, 
            'gz_args': gz_args, 
        }.items(),
    )

    description_launch_py = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'description.launch.py'])),
        launch_arguments={
            'urdf_package': LaunchConfiguration('urdf_package'),
            'urdf_package_path': LaunchConfiguration('urdf_package_path')}.items()
    )

    # urdf_spawner = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_spawn_model.launch.py'])),
    #         launch_arguments={
    #             'world': 'default', 
    #             'topic': '/robot_description',
    #             'entity_name': 'robot',
    #             'z': f'{0.5}', 
    #         }.items()
    #     )
    
    urdf_spawner = IncludeLaunchDescription( ##프로젝트용
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_spawn_model.launch.py'])),
            launch_arguments={
                'world': 'default',
                'topic': '/robot_description',
                'entity_name': 'robot',
                'x': f'{1.2}',
                'y': f'{-28.5}',
                'z': f'{1.5}',
                'Y': f'{1.5708}',
            }.items()
        )
    
#     90도: 1.5708
#   -90도: -1.5708
#   180도: 3.14159


    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_path
        }],
        output='screen'
    )

    # line_follower_node = Node(
    #     package='line_follower',
    #     executable='line_follower',
    #     output='screen',
    # )
    

    return LaunchDescription([
        gz_resource_env,
        gui_arg,
        world_arg,
        package_arg,
        model_arg,
        empty_world_launch,
        description_launch_py,
        urdf_spawner,
        ros_gz_bridge,
        # TimerAction(
        #         period=20.0,
        #         actions=[line_follower_node] #이거 만들어야함
        # )
    ])
