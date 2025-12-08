# Quickstart: Module 3 Content Setup

## Prerequisites

-   NVIDIA Isaac Sim installed (container or app) on a Workstation with NVIDIA RTX 4070 Ti+ (12GB+ VRAM).
-   NVIDIA Jetson Orin Nano/NX device with JetPack installed.
-   Isaac ROS installed on Jetson (e.g., via Docker containers).
-   ROS 2 (Humble or greater) installed and sourced on both Workstation and Jetson.
-   Python 3.8+ with relevant packages (e.g., `numpy`, `matplotlib`).

## Setting up Isaac Sim (Workstation)

1.  **Install Isaac Sim**: Follow NVIDIA's official documentation to install Isaac Sim via Omniverse Launcher or as a Docker container.
2.  **Launch Isaac Sim**: Start the Isaac Sim application.
3.  **Python Script Editor**: Open the built-in Python Script Editor (Window -> Scripting -> Script Editor) to run Python code.

## Setting up Isaac ROS on Jetson (Edge)

1.  **Prepare Jetson**: Ensure your Jetson Orin device is powered on and connected to the network.
2.  **Install Isaac ROS**: Follow NVIDIA's official Isaac ROS setup guides to install the necessary Docker containers and ROS 2 workspaces on your Jetson.
3.  **Build ROS 2 Packages**: Build your ROS 2 packages containing Isaac ROS nodes.

## Running Simulations and ROS 2 Nodes

1.  **Isaac Sim (Workstation)**:
    *   Execute Python scripts within Isaac Sim to run simulations, generate synthetic data, and control the scene.
2.  **Isaac ROS (Edge)**:
    *   Launch ROS 2 nodes (e.g., `isaac_ros_visual_slam`) using `ros2 launch` commands on the Jetson.
3.  **Visualization**:
    *   Use Rviz (on Workstation, or connected to the Jetson via ROS 2 network) to visualize robot pose, maps, sensor data, and other ROS 2 topics.

## Verification

1.  **Synthetic Data Generation**: Verify that the generated datasets contain the expected labels and randomization.
2.  **VSLAM**: Check Rviz for accurate robot pose and map generation from Isaac ROS VSLAM output.
3.  **Capstone**: Observe robot behavior in Isaac Sim and confirm it follows the target based on its perception.
