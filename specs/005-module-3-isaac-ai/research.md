# Research & Validation: Module 3: The AI-Robot Brain Content

**Feature**: Module 3: The AI-Robot Brain Content
**Branch**: `005-module-3-isaac-ai`
**Date**: 2025-12-08

## Resolved Unknowns

### 1. Isaac Sim Documentation

-   **Context7 ID for NVIDIA Isaac Sim documentation**:
    -   **Decision**: Use `/isaac-sim/isaacsim` as the primary source for Isaac Sim documentation.

-   **Replicator API (Python) for synthetic data generation**:
    -   **Decision**: Relevant Python snippets for domain randomization, triggering events, and simulation loops were found in `/isaac-sim/isaacsim` (specifically `isaacsim.replicator.domain_randomization` and `omni.replicator.core`).
    -   **Snippet Example (Domain Randomization)**:
        ```python
        import isaacsim.replicator.domain_randomization as dr
        import omni.replicator.core as rep

        dr.physics_view.register_simulation_context(world)
        dr.physics_view.register_rigid_prim_view(object_view)
        dr.physics_view.register_articulation_view(franka_view)

        with dr.trigger.on_rl_frame(num_envs=num_envs):
            with dr.gate.on_interval(interval=20):
                dr.physics_view.randomize_simulation_context(
                    operation="scaling",
                    gravity=rep.distribution.uniform((1, 1, 0.0), (1, 1, 2.0)),
                )
            # ... other randomization rules
        rep.orchestrator.run()
        ```

-   **Python API for controlling Isaac Sim camera and getting ground-truth data**:
    -   **Decision**: Snippets found in `/isaac-sim/isaacsim` (specifically `isaacsim.sensors.camera`) show how to configure cameras, add annotators for bounding boxes/segmentation, and retrieve RGBA data.
    -   **Snippet Example (Camera Configuration & Data Retrieval)**:
        ```python
        from isaacsim.sensors.camera import Camera
        from omni.kit.viewport.utility import get_active_viewport
        import omni.replicator.core as rep

        viewport_api = get_active_viewport()
        render_product_path = viewport_api.get_render_product_path()
        camera = Camera(
            prim_path="/World/camera",
            position=np.array([0.0, 0.0, 25.0]),
            resolution=(1280, 720),
            render_product_path = render_product_path
        )
        # ... initialize camera
        camera.add_bounding_box_2d_tight_to_frame()
        # ...
        rgb_data = camera.get_rgba()
        ```

-   **USD (Universal Scene Description) workflows within Omniverse**:
    -   **Decision**: Relevant snippets for MJCF import to USD, setting up physics on USD stage, and basic USD prim structure were found in `/isaac-sim/isaacsim`.

### 2. Isaac ROS Documentation

-   **Context7 ID for NVIDIA Isaac ROS documentation**:
    -   **Decision**: Use `/nvidia-isaac-ros/isaac_perceptor` as the closest general `context7` ID for Isaac ROS packages. Acknowledge its focus is more on build infrastructure.

-   **VSLAM (Visual SLAM) and Nvblox concepts**:
    -   **Decision**: Direct conceptual descriptions or detailed configurations for VSLAM and Nvblox were not found within `/nvidia-isaac-ros/isaac_perceptor` via `context7` automated search. The content will explain the concepts drawing from general robotics knowledge and refer to official NVIDIA Isaac ROS documentation for detailed configuration.

-   **Standard launch file configuration for `isaac_ros_visual_slam`**:
    -   **Decision**: No direct Python launch file configurations for `isaac_ros_visual_slam` were found within `/nvidia-isaac-ros/isaac_perceptor` via `context7` automated search. The content will discuss the *concept* of such a launch file and guide the user to external official NVIDIA Isaac ROS documentation for specific examples.

### 3. NVIDIA Jetson Documentation

-   **Context7 ID for NVIDIA Jetson Orin documentation**:
    -   **Decision**: Use `/dusty-nv/jetson-inference` as the primary source for Jetson-related hardware acceleration.

-   **"Hardware Acceleration" (CUDA/TensorRT) for Edge processing**:
    -   **Decision**: C++ snippets for loading TensorRT engines, specifying `deviceType` (GPU/DLA), and CUDA-accelerated image processing were found in `/dusty-nv/jetson-inference`.
    -   **Snippet Example (TensorRT Engine Load)**:
        ```cpp
        bool LoadEngine(nvinfer1::ICudaEngine* engine, const std::vector<std::string>& input_blobs, const std::vector<std::string>& output_blobs, deviceType device = DEVICE_GPU, cudaStream_t stream = NULL);
        ```

### 4. Nav2 Integration

-   **How Isaac ROS VSLAM typically feeds data into the standard ROS 2 Nav2 stack**:
    -   **Decision**: This will be described conceptually in the content (e.g., VSLAM outputting robot pose and map data that Nav2 consumes). Specific launch parameters would typically reside in the `isaac_ros_visual_slam` launch files (which are currently elusive from direct `context7` search).

## Remaining Unknowns / Caveats

-   Specific, ready-to-use Python launch file configurations for `isaac_ros_visual_slam` and detailed C# setup for ROS-TCP Connector (for Module 2) are not directly available via automated `context7` library ID searches, as these tend to be within broader official documentation or specific example repositories. The content will guide the user to these external official sources as authoritative.
