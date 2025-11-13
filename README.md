# gazebo_rviz_demo: ROS 2 Humble, Gazebo, and RViz Integration Demo

This package is a simple demonstration of how to integrate ROS 2 Humble, Gazebo, and RViz.

It launches a simple one-joint robotic arm (`simple_arm`) in a Gazebo simulation environment. Simultaneously, it launches RViz to display the 3D model of the robot.

The robot's state in Gazebo (joint positions) is published to the `/joint_states` topic. The `robot_state_publisher` node uses this information to publish `/tf` transforms, allowing RViz to accurately visualize the model.

## 📋 Prerequisites

Before you begin, ensure you have the following installed:

1.  **ROS 2 Humble:** The `ros-humble-desktop-full` version is recommended.
2.  **Gazebo and its ROS 2 plugins:**
    ```bash
    sudo apt update
    sudo apt install gazebo libgazebo-dev
    sudo apt install ros-humble-gazebo-ros-pkgs
    ```

## 📁 File Structure

This package uses resource files organized as follows:

gazebo_rviz_demo/ ├── launch/ │ └── demo.launch.py # Main launch file, starts everything ├── urdf/ │ └── simple_arm.urdf # Robot description (geometry, joints, Gazebo plugin) ├── worlds/ │ └── empty_world.world # Gazebo world file (ground plane and sun) ├── rviz/ │ └── demo.rviz # RViz configuration (displays RobotModel and TF) ├── package.xml # Package manifest and dependencies └── setup.py # Setup script (crucial for copying resource files)

## 🛠️ How to Build

Assuming you have created this package inside `~/ros2_ws/src`:

1.  Open a terminal and navigate to your workspace's root directory:
    ```bash
    cd ~/ros2_ws
    ```

2.  Build the package:
    ```bash
    colcon build --packages-select gazebo_rviz_demo
    ```

## 🚀 How to Run

1.  Source your workspace (do this in every new terminal):
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2.  Run the launch file:
    ```bash
    ros2 launch gazebo_rviz_demo demo.launch.py
    ```

## 🎆 Expected Outcome

After running the launch command, two windows should open automatically:

1.  **Gazebo:** Shows the robot (white box + blue cylinder) standing on a ground plane.
2.  **RViz:** Shows the same robot model, mirroring its pose from Gazebo.

---

## 🔧 Troubleshooting

### Error: `PackageNotFoundError: "package 'gazebo_ros' not found"`

This is the most common error when running this demo.

* **Full Error:**
    ```
    [ERROR] [launch]: Caught exception in launch...
    PackageNotFoundError: "package 'gazebo_ros' not found, searching: [...]"
    ```

* **Cause:**
    You have installed Gazebo, but you have not installed the "bridge" packages (`gazebo_ros`, `gazebo_plugins`, etc.) that allow ROS 2 to communicate with Gazebo.

* **Solution:**
    Run the following command to install the ROS 2 Humble integration packages for Gazebo:
    ```bash
    sudo apt install ros-humble-gazebo-ros-pkgs
    ```

* **Important:** After the installation finishes, **close your current terminal** and open a new one. Then, rebuild and re-run the demo. This ensures your terminal session recognizes the newly installed packages.
