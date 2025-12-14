---
sidebar_position: 4
title: Digital Twin Simulation
description: Build virtual robots with Gazebo and NVIDIA Isaac Sim
---

# Digital Twin Simulation

Digital twins are virtual replicas of physical robots, enabling safe development, testing, and training. This chapter covers simulation with Gazebo and NVIDIA Isaac Sim.

## Why Simulation?

Simulation provides critical advantages:

1. **Safety** - Test dangerous scenarios without risk
2. **Speed** - Run faster than real-time
3. **Cost** - No hardware wear or damage
4. **Reproducibility** - Repeat exact conditions
5. **Scalability** - Run thousands of parallel instances

### The Simulation Reality Gap

The challenge: simulated behavior doesn't perfectly match reality.

| Factor | Simulation | Reality |
|--------|------------|---------|
| Physics | Approximated | Complex |
| Sensors | Clean data | Noisy |
| Actuators | Instant response | Delays, friction |
| Environment | Simplified | Unpredictable |

Bridging techniques:
- **Domain randomization** - Vary simulation parameters
- **System identification** - Match sim to real measurements
- **Sim-to-real transfer** - Train in sim, fine-tune on hardware

## Gazebo Simulation

Gazebo is the most widely used open-source robot simulator.

### Gazebo Architecture

```
┌─────────────────────────────────────────┐
│              Gazebo Server              │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  │
│  │ Physics │  │ Sensors │  │ Plugins │  │
│  │ Engine  │  │  Sim    │  │         │  │
│  └─────────┘  └─────────┘  └─────────┘  │
└─────────────────────────────────────────┘
              ↕ Transport Layer
┌─────────────────────────────────────────┐
│              Gazebo Client              │
│  ┌─────────────────────────────────┐    │
│  │      3D Visualization GUI       │    │
│  └─────────────────────────────────┘    │
└─────────────────────────────────────────┘
```

### URDF: Robot Description

URDF (Unified Robot Description Format) defines robot structure:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01"/>
    </inertial>
  </link>

  <!-- Revolute joint -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
</robot>
```

### SDF: Simulation Description

SDF extends URDF with simulation-specific features:

```xml
<sdf version="1.9">
  <world name="default">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <model name="my_robot">
      <!-- Robot definition -->
    </model>
  </world>
</sdf>
```

### Gazebo Plugins

Plugins extend Gazebo functionality:

```xml
<gazebo>
  <!-- Differential drive controller -->
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <ros>
      <namespace>/robot</namespace>
    </ros>
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.4</wheel_separation>
    <wheel_diameter>0.2</wheel_diameter>
  </plugin>
</gazebo>
```

Common plugins:
- `gazebo_ros_camera` - Camera sensor
- `gazebo_ros_imu_sensor` - IMU sensor
- `gazebo_ros_lidar` - LiDAR sensor
- `gazebo_ros_joint_state_publisher` - Joint states

## NVIDIA Isaac Sim

Isaac Sim provides GPU-accelerated, photorealistic simulation.

### Isaac Sim Features

| Feature | Benefit |
|---------|---------|
| RTX Rendering | Photorealistic visuals |
| PhysX 5 | Advanced physics simulation |
| Synthetic Data | Automatic ground truth labels |
| ROS 2 Bridge | Direct ROS 2 integration |
| Python API | Scriptable workflows |
| Cloud Deployment | Scalable training |

### Isaac Sim Architecture

```
┌─────────────────────────────────────────┐
│         NVIDIA Isaac Sim                │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  │
│  │ Omni-   │  │ PhysX   │  │  RTX    │  │
│  │ verse   │  │ Physics │  │ Render  │  │
│  └─────────┘  └─────────┘  └─────────┘  │
└─────────────────────────────────────────┘
              ↕ OmniGraph
┌─────────────────────────────────────────┐
│           Isaac Extensions              │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  │
│  │  ROS 2  │  │Synthetic│  │ Robot   │  │
│  │ Bridge  │  │  Data   │  │ Models  │  │
│  └─────────┘  └─────────┘  └─────────┘  │
└─────────────────────────────────────────┘
```

### Python Scripting

Control Isaac Sim programmatically:

```python
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage

# Create world
world = World()

# Add robot
add_reference_to_stage(
    usd_path="/path/to/robot.usd",
    prim_path="/World/Robot"
)
robot = world.scene.add(Robot(prim_path="/World/Robot", name="my_robot"))

# Simulation loop
world.reset()
while simulation_app.is_running():
    world.step(render=True)
```

### Synthetic Data Generation

Isaac Sim generates training data automatically:

- **RGB images** with camera intrinsics
- **Depth maps** for 3D perception
- **Semantic segmentation** labels
- **Instance segmentation** for object detection
- **2D/3D bounding boxes**
- **Skeleton data** for pose estimation

## Physics Engines

Both simulators offer physics engine choices:

### Gazebo Physics Engines

- **ODE** - Open Dynamics Engine (default)
- **Bullet** - Popular game physics
- **DART** - Accurate for articulated robots
- **Simbody** - Biomechanics focus

### Isaac Sim PhysX 5

PhysX 5 features:
- GPU acceleration
- Soft body simulation
- Fluid dynamics
- Particle systems
- Deformable objects

## Sensor Simulation

### Camera Sensors

```xml
<!-- Gazebo camera -->
<sensor name="camera" type="camera">
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
</sensor>
```

### LiDAR Sensors

```xml
<!-- Gazebo LiDAR -->
<sensor name="lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14</min_angle>
        <max_angle>3.14</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>10.0</max>
    </range>
  </ray>
</sensor>
```

## Summary

Key simulation concepts:
- Digital twins enable safe, fast development
- Gazebo is open-source and widely used
- Isaac Sim offers photorealistic GPU simulation
- URDF/SDF describe robot and world models
- Plugins add sensors and controllers
- Bridging the sim-to-real gap is essential

---

← **Previous:** [ROS 2 Fundamentals](./ros2-fundamentals.md) | **Next:** [Vision-Language-Action Systems](./vla-systems.md) →
