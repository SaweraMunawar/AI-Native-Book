---
sidebar_position: 2
title: Basics of Humanoid Robotics
description: Understanding humanoid robot anatomy, kinematics, and fundamental concepts
---

# Basics of Humanoid Robotics

This chapter covers the fundamental concepts of humanoid robot design, from mechanical anatomy to basic motion control.

## Humanoid Robot Anatomy

A humanoid robot is designed to replicate human form and function. Understanding its anatomy is essential for programming and control.

### Skeletal Structure

The robot's frame provides:
- **Structural support** for all components
- **Joint articulation** for movement
- **Mounting points** for sensors and actuators

Typical humanoid robots have 20-40 degrees of freedom (DoF), compared to ~200 in the human body.

### Degrees of Freedom by Body Part

| Body Region | Typical DoF | Human DoF |
|-------------|-------------|-----------|
| Head/Neck | 2-3 | 7 |
| Each Arm | 6-7 | 7 |
| Torso | 2-3 | 3 |
| Each Leg | 6-7 | 6 |
| Each Hand | 4-20 | 27 |

### Actuators

Humanoid robots use various actuator types:

1. **Electric Motors**
   - DC motors with gearboxes
   - Brushless DC (BLDC) motors
   - Servo motors for precise positioning

2. **Series Elastic Actuators (SEA)**
   - Spring element between motor and load
   - Improved force control and safety
   - Used in legs for compliant walking

3. **Hydraulic Actuators**
   - High power-to-weight ratio
   - Used in Boston Dynamics robots
   - More complex maintenance

## Kinematics

Kinematics describes robot motion without considering forces.

### Forward Kinematics

Given joint angles, calculate end-effector position:

```
Position = f(θ₁, θ₂, ..., θₙ)
```

For a 2-link arm:
```python
x = L1 * cos(θ1) + L2 * cos(θ1 + θ2)
y = L1 * sin(θ1) + L2 * sin(θ1 + θ2)
```

### Inverse Kinematics

Given desired end-effector position, calculate required joint angles:

```
(θ₁, θ₂, ..., θₙ) = f⁻¹(x, y, z)
```

Inverse kinematics is more challenging because:
- Multiple solutions may exist
- Some positions are unreachable
- Singularities cause computational issues

### Denavit-Hartenberg Parameters

The DH convention standardizes how we describe robot links:

| Parameter | Symbol | Description |
|-----------|--------|-------------|
| Link length | a | Distance along x-axis |
| Link twist | α | Rotation about x-axis |
| Link offset | d | Distance along z-axis |
| Joint angle | θ | Rotation about z-axis |

## Sensors

Humanoid robots rely on multiple sensor modalities:

### Proprioceptive Sensors

Internal state sensors:
- **Encoders** - Joint position measurement
- **Gyroscopes** - Angular velocity
- **Accelerometers** - Linear acceleration
- **IMU** - Combined inertial measurement

### Exteroceptive Sensors

Environment perception:
- **Cameras** - Visual perception (RGB, depth)
- **LiDAR** - 3D point cloud mapping
- **Force/Torque sensors** - Contact detection
- **Tactile sensors** - Surface interaction

## Balance and Locomotion

Walking is one of the most challenging aspects of humanoid robotics.

### Zero Moment Point (ZMP)

The ZMP is the point on the ground where the total moment of ground reaction forces is zero. For stable walking:

- ZMP must stay within the support polygon
- Moving ZMP too fast causes tipping
- ZMP trajectory planning is essential

### Walking Gaits

Common bipedal walking approaches:

1. **Static Walking** - CoM always over support polygon (slow, stable)
2. **Dynamic Walking** - Controlled falling (faster, human-like)
3. **Running** - Flight phase with no ground contact

### The Inverted Pendulum Model

Simplified model treating the robot as a mass on a stick:

```
CoM dynamics: M * ẍ = M * g * (x - p) / z
```

Where:
- `x` = CoM horizontal position
- `p` = ZMP position
- `z` = CoM height
- `g` = gravity

## Manipulation

Humanoid arms and hands enable object interaction.

### Grasp Types

1. **Power Grasp** - Whole hand wraps around object
2. **Precision Grasp** - Fingertips only
3. **Pinch Grasp** - Two fingers opposing

### Motion Planning

Path planning for manipulation:
- **Configuration space** - Joint angle space
- **Workspace** - Cartesian end-effector space
- **Collision detection** - Self and environment

## Summary

Key concepts from this chapter:

- Humanoid anatomy includes skeleton, actuators, and sensors
- Kinematics relates joint angles to positions
- Balance requires ZMP control
- Manipulation involves grasp planning and motion control

---

← **Previous:** [Introduction](./intro.md) | **Next:** [ROS 2 Fundamentals](./ros2-fundamentals.md) →
