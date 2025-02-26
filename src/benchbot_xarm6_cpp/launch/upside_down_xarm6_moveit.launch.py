from math import pi
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder

def rosbridge_launch_setup():
    import os
    rosbridge_server_dir = FindPackageShare('rosbridge_server')
    rosbridge_launch_path = PathJoinSubstitution([rosbridge_server_dir, 'launch', 'rosbridge_tcp_launch.xml'])
    
    return IncludeLaunchDescription(
                XMLLaunchDescriptionSource(rosbridge_launch_path)
            )
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
from launch.logging import get_logger
from ament_index_python.packages import get_package_share_directory
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):

    attach_to = LaunchConfiguration('attach_to', default='world')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default=f'"0 {pi} 0"')
    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            context=LaunchContext(),
            controllers_name="fake_controllers",
            robot_type="xarm",
            dof=6,
            ros2_control_plugin="uf_robot_hardware/UFRobotFakeSystemHardware",
            
            attach_to=attach_to,
            attach_xyz=attach_xyz,
            attach_rpy=attach_rpy,
            # add_other_geometry=False,
            # geometry_type="mesh",
            # geometry_mesh_filename="../../../../../../my_stuff/meshes/extruder_assembly.stl",
            # geometry_mesh_origin_rpy='"0 0 3.655"',
            # geometry_mesh_origin_xyz='"0.0203 0.0886 0"',
            # geometry_mesh_tcp_xyz='"0 0 0.1124"',
            # geometry_mesh_tcp_rpy='"0 0 0"',
            # geometry_mass=0.8,
        )
        .robot_description()
        .trajectory_execution(file_path="config/xarm6/fake_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--log-level", "WARN"],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("xarm_moveit_config") + "/rviz/moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("xarm_controller"),
        "config",
        "xarm6_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path, moveit_config.robot_description],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["xarm6_traj_controller", "-c", "/controller_manager"],
    )

    return[
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
        ]
# def launch_setup(context, *args, **kwargs):
#     xacro_file = LaunchConfiguration('xacro_file', default=PathJoinSubstitution([FindPackageShare('benchbot_xarm6_description'), 'urdf', 'upside_down_xarm6.urdf.xacro']))
#     attach_to = LaunchConfiguration('attach_to', default='world')
#     attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 2"')
#     attach_rpy = LaunchConfiguration('attach_rpy', default=f'"0 {pi} 0"')

#     prefix = LaunchConfiguration('prefix', default='')
#     hw_ns = LaunchConfiguration('hw_ns', default='xarm')
#     limited = LaunchConfiguration('limited', default=True)
#     effort_control = LaunchConfiguration('effort_control', default=False)
#     velocity_control = LaunchConfiguration('velocity_control', default=False)
#     add_gripper = LaunchConfiguration('add_gripper', default=False)
#     add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
#     add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
#     dof = LaunchConfiguration('dof', default=6)
#     robot_type = LaunchConfiguration('robot_type', default='xarm')
#     no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)

#     add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
#     add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
#     model1300 = LaunchConfiguration('model1300', default=False)

#     add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
#     geometry_type = LaunchConfiguration('geometry_type', default='box')
#     geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
#     geometry_height = LaunchConfiguration('geometry_height', default=0.1)
#     geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
#     geometry_length = LaunchConfiguration('geometry_length', default=0.1)
#     geometry_width = LaunchConfiguration('geometry_width', default=0.1)
#     geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
#     geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
#     geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
#     geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
#     geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')

#     kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')

#     ros2_control_plugin = 'uf_robot_hardware/UFRobotFakeSystemHardware'
#     controllers_name = 'fake_controllers'
#     moveit_controller_manager_key = 'moveit_simple_controller_manager'
#     moveit_controller_manager_value = 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
#     xarm_type = '{}{}'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')
#     ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)
    
#     # robot description launch
#     # xarm_description/launch/_robot_description.launch.py
#     robot_description_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_description'), 'launch', '_robot_description.launch.py'])),
#         launch_arguments={
#             'xacro_file': xacro_file,
#             'attach_to': attach_to,
#             'attach_xyz': attach_xyz,
#             'attach_rpy': attach_rpy,
            
#             'prefix': prefix,
#             'hw_ns': hw_ns,
#             'limited': limited,
#             'effort_control': effort_control,
#             'velocity_control': velocity_control,
#             'add_gripper': add_gripper,
#             'add_vacuum_gripper': add_vacuum_gripper,
#             'add_bio_gripper': add_bio_gripper,
#             'dof': dof,
#             'robot_type': robot_type,
#             'ros2_control_plugin': ros2_control_plugin,
#             'joint_states_remapping': 'joint_states',
#             'add_realsense_d435i': add_realsense_d435i,
#             'add_d435i_links': add_d435i_links,
#             'model1300': model1300,
#             'add_other_geometry': add_other_geometry,
#             'geometry_type': geometry_type,
#             'geometry_mass': geometry_mass,
#             'geometry_height': geometry_height,
#             'geometry_radius': geometry_radius,
#             'geometry_length': geometry_length,
#             'geometry_width': geometry_width,
#             'geometry_mesh_filename': geometry_mesh_filename,
#             'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,
#             'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,
#             'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,
#             'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,
#             'kinematics_suffix': kinematics_suffix,
#         }.items(),
#     )

#     # robot moveit common launch
#     # xarm_moveit_config/launch/_robot_moveit_common.launch.py
#     robot_moveit_common_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_common.launch.py'])),
#         launch_arguments={
#             'prefix': prefix,
#             'hw_ns': hw_ns,
#             'limited': limited,
#             'effort_control': effort_control,
#             'velocity_control': velocity_control,
#             'add_gripper': add_gripper,
#             # 'add_gripper': add_gripper if robot_type.perform(context) == 'xarm' else 'false',
#             'add_vacuum_gripper': add_vacuum_gripper,
#             'add_bio_gripper': add_bio_gripper,
#             'dof': dof,
#             'robot_type': robot_type,
#             'no_gui_ctrl': no_gui_ctrl,
#             'ros2_control_plugin': ros2_control_plugin,
#             'controllers_name': controllers_name,
#             'moveit_controller_manager_key': moveit_controller_manager_key,
#             'moveit_controller_manager_value': moveit_controller_manager_value,
#             'add_realsense_d435i': add_realsense_d435i,
#             'add_d435i_links': add_d435i_links,
#             'model1300': model1300,
#             'add_other_geometry': add_other_geometry,
#             'geometry_type': geometry_type,
#             'geometry_mass': geometry_mass,
#             'geometry_height': geometry_height,
#             'geometry_radius': geometry_radius,
#             'geometry_length': geometry_length,
#             'geometry_width': geometry_width,
#             'geometry_mesh_filename': geometry_mesh_filename,
#             'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,
#             'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,
#             'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,
#             'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,
#             'kinematics_suffix': kinematics_suffix,
#         }.items(),
#     )

#     remappings = [
#         ('follow_joint_trajectory', '{}{}_traj_controller/follow_joint_trajectory'.format(prefix.perform(context), xarm_type)),
#     ]
#     controllers = ['{}{}_traj_controller'.format(prefix.perform(context), xarm_type)]
#     if add_gripper.perform(context) in ('True', 'true') and robot_type.perform(context) != 'lite':
#         remappings.append(
#             ('follow_joint_trajectory', '{}{}_gripper_traj_controller/follow_joint_trajectory'.format(prefix.perform(context), robot_type.perform(context)))
#         )
#         controllers.append('{}{}_gripper_traj_controller'.format(prefix.perform(context), robot_type.perform(context)))
#     elif add_bio_gripper.perform(context) in ('True', 'true') and robot_type.perform(context) != 'lite':
#         remappings.append(
#             ('follow_joint_trajectory', '{}bio_gripper_traj_controller/follow_joint_trajectory'.format(prefix.perform(context)))
#         )
#         controllers.append('{}bio_gripper_traj_controller'.format(prefix.perform(context)))
#     # joint state publisher node
#     joint_state_publisher_node = Node(
#         package='joint_state_publisher',
#         executable='joint_state_publisher',
#         name='joint_state_publisher',
#         output='screen',
#         parameters=[{'source_list': ['joint_states']}],
#         remappings=remappings,
#     )

#     # ros2 control launch
#     # xarm_controller/launch/_ros2_control.launch.py
#     ros2_control_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_controller'), 'launch', '_ros2_control.launch.py'])),
#         launch_arguments={
#             'prefix': prefix,
#             'hw_ns': hw_ns,
#             'limited': limited,
#             'effort_control': effort_control,
#             'velocity_control': velocity_control,
#             'add_gripper': add_gripper,
#             'add_vacuum_gripper': add_vacuum_gripper,
#             'add_bio_gripper': add_bio_gripper,
#             'dof': dof,
#             'robot_type': robot_type,
#             'ros2_control_plugin': ros2_control_plugin,
#             'add_realsense_d435i': add_realsense_d435i,
#             'add_d435i_links': add_d435i_links,
#             'model1300': model1300,
#             'add_other_geometry': add_other_geometry,
#             'geometry_type': geometry_type,
#             'geometry_mass': geometry_mass,
#             'geometry_height': geometry_height,
#             'geometry_radius': geometry_radius,
#             'geometry_length': geometry_length,
#             'geometry_width': geometry_width,
#             'geometry_mesh_filename': geometry_mesh_filename,
#             'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,
#             'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,
#             'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,
#             'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,
#             'kinematics_suffix': kinematics_suffix,
#         }.items(),
#     )

#     # Load controllers
#     load_controllers = []
#     for controller in controllers:
#         load_controllers.append(Node(
#             package='controller_manager',
#             executable='spawner',
#             output='screen',
#             arguments=[
#                 controller,
#                 '--controller-manager', '{}/controller_manager'.format(ros_namespace)
#             ],
#         ))

#     return [
#         robot_description_launch,
#         robot_moveit_common_launch,
#         joint_state_publisher_node,
#         ros2_control_launch,
#     ] + load_controllers


def generate_launch_description():
    return LaunchDescription([
        rosbridge_launch_setup(),
        OpaqueFunction(function=launch_setup)
    ])