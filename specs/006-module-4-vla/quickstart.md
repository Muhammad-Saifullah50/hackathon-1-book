# Quickstart: Module 4 Content Setup

## Prerequisites

-   ROS 2 (Humble or greater) installed and sourced.
-   Python 3.8+ with `rclpy`, `pyaudio` (or `sounddevice`), `openai` (for OpenAI API client).
-   NVIDIA Isaac Sim / Gazebo (for Capstone simulation).
-   NVIDIA Jetson Orin (for Edge deployment, if testing Whisper Tiny/Base).
-   OpenAI API Key (if using real LLM API), managed securely (e.g., environment variable).
-   `ikpy` Python library installed for Inverse Kinematics.

## Setting up Voice Interface (Workstation/Jetson)

1.  **Install Python Libraries**:
    ```bash
    pip install pyaudio # Or sounddevice if preferred
    pip install openai-whisper
    pip install "openai>=1.0.0" # For OpenAI Python client
    ```
2.  **ROS 2 Workspace**: Create a ROS 2 package for your "Listening Node."
3.  **Audio Setup**: Configure your microphone.
4.  **Wake Word Detection**: Implement or integrate a wake word detection library (or a simple voice activity detection).

## Setting up Cognitive Planning (Workstation/Cloud)

1.  **Environment Variable**: Set your OpenAI API key:
    ```bash
    export OPENAI_API_KEY="YOUR_API_KEY"
    ```
2.  **ROS 2 Workspace**: Create a ROS 2 package for your "Planner Node."

## Setting up Humanoid Mechanics (Workstation/Edge)

1.  **IKPy Installation**:
    ```bash
    pip install ikpy
    ```
2.  **URDF/Robot Model**: Ensure you have a URDF file for your humanoid robot (e.g., Unitree G1) that IKPy can load.

## Running the Capstone Simulation ("Butler Bot")

1.  **Launch Simulation**: Start your simulated environment (Isaac Sim/Gazebo) with your humanoid robot.
2.  **Launch Nodes**:
    *   Run the `Voice Node` (Listening Node) to transcribe spoken commands.
    *   Run the `Planner Node` to process text and send Action Goals to the robot.
    *   Run the robot's control nodes (Navigation, Vision, Manipulation) that act as ROS 2 Action Servers.

## Verification

1.  **Voice Node**: Verify that the `/human_command` topic publishes correct transcriptions of spoken words.
    ```bash
    ros2 topic echo /human_command
    ```
2.  **Planner Node**: Verify that the `/robot_goal` (or similar) topic receives correct JSON commands from the LLM.
    ```bash
    ros2 topic echo /robot_goal
    ```
3.  **Robot Behavior**: Observe the robot's actions in the simulation and confirm it performs the requested tasks (e.g., moves to kitchen, identifies apple).
