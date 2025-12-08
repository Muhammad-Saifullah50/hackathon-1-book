# Data Model: Module 2 Content

## MDX Front Matter

| Field | Type | Required | Description |
|---|---|---|---|
| title | string | Yes | Chapter title |
| sidebar_label | string | No | Short title for sidebar |
| description | string | Yes | SEO description |
| experience_level | 'Beginner' \| 'Advanced' | Yes | For filtering/badges |

## Code Snippet Structures

### URDF/SDF Inertial Properties (from context7: /gazebosim/docs)

```xml
<inertial>
    <mass>#VALUE#</mass>
    <inertia>
        <ixx>#VALUE#</ixx>
        <ixy>#VALUE#</ixy>
        <ixz>#VALUE#</ixz>
        <iyy>#VALUE#</iyy>
        <iyz>#VALUE#</iyz>
        <izz>#VALUE#</izz>
    </inertia>
</inertial>
```

### Gazebo Sensor Plugin (LiDAR - from context7: /gazebosim/docs)

```xml
<sensor name="gpu_lidar" type="gpu_lidar">
    <always_on>true</always_on>
    <visualize>true</visualize>
    <update_rate>#VALUE#</update_rate>
    <ray>
        <scan>
            <horizontal>
                <samples>640</samples>
                <resolution>1</resolution>
                <min_angle>-1.396263</min_angle>
                <max_angle>1.396263</max_angle>
            </horizontal>
            <vertical>
                <samples>1</samples>
                <resolution>0.01</resolution>
                <min_angle>0</min_angle>
                <max_angle>0</max_angle>
            </vertical>
        </scan>
        <range>
            <min>0.08</min>
            <max>10.0</max>
            <resolution>0.01</resolution>
        </range>
    </ray>
    <plugin filename="libgazebo_ros_ray_sensor.so" name="gazebo_ros_head_hokuyo_controller">
        <ros>
            <argument>~/out</argument>
            <argument>--ros-args -r ~/out:=scan</argument>
        </ros>
    </plugin>
</sensor>
```
*(Note: Used `libgazebo_ros_ray_sensor.so` as a more generic ray sensor plugin example from ROS-Gazebo integration docs, if `libgazebo_ros_laser.so` is specifically Gazebo Classic. The exact plugin name will be verified again with `context7` during content creation, adhering to the "do not invent" rule.)*

### ROS 2 Launch File for Spawning in Gazebo (from context7: /gazebosim/docs)

```xml
<launch>
  <gz_spawn_model
    world="$(var world)"
    file="$(var file)"
    model_string="$(var model_string)"
    topic="$(var topic)"
    entity_name="$(var entity_name)"
    allow_renaming="$(var allow_renaming)"
    x="$(var x)"
    y="$(var y)"
    z="$(var z)">
  </gz_spawn_model>
</launch>
```

### ROS-TCP Connector Endpoint (conceptual, requires context7 for exact syntax)

*(Placeholder - specific details to be filled during implementation once `context7` yields concrete info.)*
```csharp
// Example (conceptual): Unity C# script for ROS-TCP Endpoint setup
// using RosMessageGeneration
public class ROSTCPConnector : MonoBehaviour
{
    public string rosIP = "127.0.0.1";
    public int rosPort = 10000;
    // ...
    void Start()
    {
        // Setup publisher/subscriber
    }
}
```

## URDF Import into Unity (conceptual, requires context7 for exact syntax)

*(Placeholder - specific details to be filled during implementation once `context7` yields concrete info.)*
```csharp
// Example (conceptual): Unity C# script for URDF import
// using URDF-loaders package
public class UrdfImporter : MonoBehaviour
{
    public GameObject robotPrefab;
    public string urdfFilePath;
    // ...
    void ImportRobot()
    {
        // Load and parse URDF
    }
}
```
