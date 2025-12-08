# Quickstart: Module 2 Content Setup

## Prerequisites

-   ROS 2 (Humble or greater) installed and sourced.
-   Gazebo Sim installed.
-   Unity Hub and Unity Editor (version 2022.3 LTS or newer recommended) installed.
-   `ros_gz_sim` package installed for ROS 2 - Gazebo integration.
-   Unity Robotics packages (URDF Importer, ROS-TCP Connector) installed in Unity.

## Setting up Gazebo

1.  Launch a basic empty Gazebo world:
    ```bash
    ign gazebo -r empty.sdf
    ```
    (Or `ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=-r empty.sdf`)

2.  To spawn a robot from a URDF/SDF file:
    ```bash
    ros2 launch ros_gz_sim ros_gz_spawn_model.launch.py entity_name:=my_robot file:=$(find my_robot_description)/urdf/my_robot.urdf
    ```
    (Adjust `my_robot_description` and `my_robot.urdf` to your robot's path).

## Setting up Unity

1.  Create a new 3D Unity project.
2.  Install Unity Robotics packages (e.g., URDF Importer, ROS-TCP Connector) via Package Manager.
3.  Import your URDF model into Unity using the URDF Importer.
4.  Configure the ROS-TCP Connector in Unity to establish communication with ROS 2.

## Verification

1.  **Gazebo**: Launch a world with your robot and verify physics, visuals, and sensor data (e.g., using `ros2 topic echo /scan`).
2.  **Unity**: Run your Unity scene, verify URDF model is loaded, and ROS 2 communication (e.g., publishing data from Unity to a ROS 2 topic) is working.
