import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
# import xacro

def generate_launch_description():

    # --- 1. Lấy đường dẫn các file ---
    pkg_path = get_package_share_directory('gazebo_rviz_demo')

    # Đường dẫn file URDF
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'simple_arm.urdf')

    # Đường dẫn file World
    world_file_path = os.path.join(pkg_path, 'worlds', 'empty_world.world')

    # Đường dẫn file RViz
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'demo.rviz')

    # Đường dẫn tới launch file của Gazebo
    gazebo_launch_path = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')

    # --- 2. Đọc nội dung file URDF ---
    # (Không cần xacro vì là file urdf thuần túy)
    with open(urdf_file_path, 'r') as f:
        robot_description_content = f.read()

    # --- 3. Cấu hình các Node sẽ chạy ---

    # Node (1): robot_state_publisher
    # Node này đọc file URDF và lắng nghe /joint_states,
    # sau đó public các phép biến đổi (transforms) lên /tf
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content,
                     'use_sim_time': True}] # Rất quan trọng khi dùng Gazebo
    )

    # Node (2): Gazebo
    # Khởi chạy Gazebo, sử dụng 'world_file_path'
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={'world': world_file_path, 'verbose': 'true'}.items(),
    )

    # Node (3): Spawner
    # Node này dùng để "thả" (spawn) robot từ 'robot_description' vào Gazebo
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'simple_arm'],
        output='screen'
    )

    # Node (4): RViz
    # Khởi chạy RViz với file cấu hình 'demo.rviz'
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # --- 4. Trả về LaunchDescription ---
    return LaunchDescription([
        node_robot_state_publisher,
        start_gazebo_cmd,
        spawn_entity_cmd,
        start_rviz_cmd,
    ])


# Tom tat

# robot_state_publisher: Public /tf.

# gazebo.launch.py: Chạy mô phỏng Gazebo.

# spawn_entity.py: Thả robot vào Gazebo.

# rviz2: Chạy RViz để hiển thị.