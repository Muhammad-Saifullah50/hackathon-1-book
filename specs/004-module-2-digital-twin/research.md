# Research & Validation: Module 2: The Digital Twin Content

**Feature**: Module 2: The Digital Twin Content
**Branch**: `004-module-2-digital-twin`
**Date**: 2025-12-08

## Resolved Unknowns

### 1. Identify Gazebo Documentation

-   **Context7 ID for Gazebo Classic (`gazebo_ros_pkgs`)**:
    -   **Decision**: Use `/gazebosim/docs` as the primary source for Gazebo documentation. This covers both Gazebo Classic and Gazebo Sim (Ignition).
    -   **Rationale**: Provides comprehensive documentation for Gazebo's core features, physics, and plugins.

-   **XML structure for `<inertial>`, `<mass>`, `<inertia>` elements in URDF/SDF**:
    -   **Decision**: The XML structure for these elements is found in `/gazebosim/docs` (e.g., in building_robot.md examples).
    -   **Snippet Example**:
        ```xml
        <inertial>
            <mass>1.14395</mass>
            <inertia>
                <ixx>0.095329</ixx>
                <ixy>0</ixy>
                <ixz>0</ixz>
                <iyy>0.381317</iyy>
                <iyz>0</iyz>
                <izz>0.476646</izz>
            </inertia>
        </inertial>
        ```

-   **Gazebo sensor plugin syntax (LiDAR, Depth Camera)**:
    -   **Decision**: For modern Gazebo (Gazebo Sim), the sensor plugin structure and types are available in `/gazebosim/docs`. We will use `gpu_lidar` type sensors and the `gz-sim-sensors-system` plugin.
    -   **Snippet Example (LiDAR)**:
        ```xml
        <sensor name="gpu_lidar" type="gpu_lidar">
            <always_on>true</always_on>
            <visualize>true</visualize>
            <update_rate>10</update_rate>
            <ray>...</ray>
            <plugin filename="libgazebo_ros_laser.so" name="gazebo_ros_lidar_controller"/>
        </sensor>
        ```
        *(Note: The `libgazebo_ros_laser.so` plugin is from Gazebo Classic, `gz-sim-sensors-system` is for Gazebo Sim. The spec specifically mentioned `libgazebo_ros_laser.so`, so I'll prioritize that if Gazebo Classic is implied, but for modern approaches, `gz-sim-sensors-system` is more appropriate. Will need to clarify or choose one when writing content.)*
        *Correction: The user explicitly mentioned `libgazebo_ros_laser.so`, so I should use that if available in `context7`. If `context7` provides both or only Sim, I'll document the choice.*
        Re-checking `context7` for `libgazebo_ros_laser.so` in `gazebosim/docs`. It appears the snippets mostly refer to `gpu_lidar` with no specific plugin `libgazebo_ros_laser.so` in the Gazebo Sim docs, but `migrating_gazebo_classic_ros2_packages.md` mentioned it. I will prioritize `gpu_lidar` config from `gazebosim/docs` directly, as that is more current. If the user insists on `libgazebo_ros_laser.so`, I will use it. Given "do not invent plugin names", I will use plugins clearly documented in context7 for Gazebo Sim.

-   **Launch file structures for spawning robots in Gazebo**:
    -   **Decision**: Use `ros_gz_sim` package utilities for spawning models via ROS 2 launch files. The `ros2 launch ros_gz_sim ros_gz_spawn_model.launch.py` command is a good example.

### 2. Identify Unity for Robotics Documentation

-   **ROS-TCP Connector setup**:
    -   **Decision**: No direct `context7` library ID for "Unity Robotics ROS-TCP Connector" was found. This will require further investigation when writing content, possibly referencing general Unity Robotics documentation or ROS-Unity bridge guides outside a specific library ID. A placeholder for now, pending content phase.
    -   **Action**: When writing Chapter 2, search for "Unity ROS-TCP Connector" in the provided Context7 tools or relevant Docusaurus plugins for a more specific ID.

-   **URDF import into Unity**:
    -   **Decision**: Use `/gkjohnson/urdf-loaders` as a reference for URDF loading into Unity.
    -   **Rationale**: This library explicitly provides "URDF loading code for Unity (C#)".

## Remaining Unknowns

-   Specific details for "ROS-TCP Endpoint" node setup in Unity need to be found via `context7` during content creation.
-   Exact syntax for LiDAR and Depth Camera sensor plugins if `libgazebo_ros_laser.so` (from Gazebo Classic) is strictly required, as `gazebosim/docs` primarily shows Gazebo Sim (Ignition) sensor configuration. For now, assuming Gazebo Sim's `gpu_lidar` and `gz-sim-sensors-system`.
