# Data Model: Module 3 Content

## MDX Front Matter

| Field | Type | Required | Description |
|---|---|---|---|
| title | string | Yes | Chapter title |
| sidebar_label | string | No | Short title for sidebar |
| description | string | Yes | SEO description |
| experience_level | 'Beginner' \| 'Advanced' | Yes | For filtering/badges |

## Code Snippet Structures

### Isaac Sim Replicator API (Python) - Synthetic Data Generation (Workstation)

```python
import isaacsim.replicator.domain_randomization as dr
import omni.replicator.core as rep

# ... setup world and views (object_view, franka_view) ...

with dr.trigger.on_rl_frame(num_envs=num_envs):
    with dr.gate.on_interval(interval=20):
        dr.physics_view.randomize_simulation_context(
            operation="scaling",
            gravity=rep.distribution.uniform((1, 1, 0.0), (1, 1, 2.0)),
        )
    # ... other randomization rules ...
rep.orchestrator.run()

# Simulation loop
# while simulation_app.is_running():
#     if world.is_playing():
#         dr.physics_view.step_randomization(reset_inds)
#         world.step(render=True)
```

### Isaac Sim Camera API (Python) - Ground-Truth Data (Workstation)

```python
from isaacsim.sensors.camera import Camera
from omni.kit.viewport.utility import get_active_viewport
import omni.replicator.core as rep
import numpy as np

# ... setup simulation_app and omni.usd.get_context().new_stage() ...

viewport_api = get_active_viewport()
render_product_path = viewport_api.get_render_product_path()
camera = Camera(
    prim_path="/World/camera",
    position=np.array([0.0, 0.0, 25.0]),
    resolution=(1280, 720),
    render_product_path = render_product_path
)
# ... initialize camera and add annotators (bounding box, segmentation) ...

# Run simulation for a few frames
# for _ in range(10):
#     simulation_app.update()

rgb_data = camera.get_rgba() # Get RGB data
# ... other ground truth data (depth, segmentation)
```

### Isaac ROS VSLAM Launch File Configuration (Python) (Edge)

*(Note: Specific launch file configuration for `isaac_ros_visual_slam` not directly found via automated `context7` search. Content will guide user to external official documentation.)*
```python
# Conceptual structure, specific parameters from official Isaac ROS documentation
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            name='visual_slam_node',
            parameters=[
                # ... parameters for input topics (stereo images), output topics (pose, map),
                #     camera intrinsics, enable_localization, enable_mapping, etc. ...
            ],
            remappings=[
                # ... topic remappings if necessary ...
            ],
            output='screen'
        )
    ])
```

### NVIDIA TensorRT Engine Loading (C++) (Edge)

```cpp
#include <NvInferRuntime.h> // For nvinfer1::ICudaEngine
// ... includes for std::vector, std::string, etc.

enum deviceType { DEVICE_GPU = 0, DEVICE_DLA, DEVICE_DLA_0 = DEVICE_DLA, DEVICE_DLA_1, NUM_DEVICES };

// Function to load TensorRT engine from an existing CUDA engine instance
bool LoadEngine(nvinfer1::ICudaEngine* engine,
                const std::vector<std::string>& input_blobs,
                const std::vector<std::string>& output_blobs,
                deviceType device = DEVICE_GPU,
                cudaStream_t stream = NULL);

// Usage example (conceptual)
// nvinfer1::ICudaEngine* myEngine = /* ... obtain engine ... */;
// std::vector<std::string> inputs = {"input_blob"};
// std::vector<std::string> outputs = {"output_blob"};
// LoadEngine(myEngine, inputs, outputs, DEVICE_GPU);
```

### USD Transform Specification

```usd
xformOp:translate
xformOp:orient
xformOp:scale
xformOpOrder = [xformOp:translate, xformOp:orient, xformOp:scale]
```
