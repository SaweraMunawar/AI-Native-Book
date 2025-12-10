---
sidebar_position: 3
title: ROS 2 Fundamentals
description: Learn the Robot Operating System 2 for building robot software
---

# ROS 2 Fundamentals

ROS 2 (Robot Operating System 2) is the de facto standard middleware for robotics development. This chapter covers its core concepts and practical usage.

## What is ROS 2?

ROS 2 is not an operating system, but a middleware framework providing:

- **Communication infrastructure** between software components
- **Hardware abstraction** for sensors and actuators
- **Tool ecosystem** for visualization, debugging, simulation
- **Package management** for code reuse

### Why ROS 2 over ROS 1?

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Communication | Custom (TCPROS) | DDS standard |
| Real-time | Limited | Supported |
| Security | None built-in | DDS security |
| Multi-robot | Complex | Native support |
| Platform | Linux only | Linux, Windows, macOS |

## Core Concepts

### Nodes

A node is a single-purpose, modular process:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Node started!')

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

Best practices:
- One node per logical function
- Nodes should be independently testable
- Use meaningful, descriptive names

### Topics

Topics enable publish-subscribe communication:

```python
# Publisher
self.publisher = self.create_publisher(String, 'topic_name', 10)
self.publisher.publish(String(data='Hello'))

# Subscriber
self.subscription = self.create_subscription(
    String,
    'topic_name',
    self.callback,
    10
)
```

Topic characteristics:
- **Asynchronous** - Publishers don't wait for subscribers
- **Many-to-many** - Multiple publishers and subscribers allowed
- **Typed** - Messages have defined structure

### Services

Services provide synchronous request-response:

```python
# Service server
self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.callback)

def callback(self, request, response):
    response.sum = request.a + request.b
    return response

# Service client
client = self.create_client(AddTwoInts, 'add_two_ints')
future = client.call_async(request)
```

Use services when:
- You need a response before continuing
- The operation is quick (< 1 second)
- The call is infrequent

### Actions

Actions handle long-running tasks with feedback:

```python
# Action server
self._action_server = ActionServer(
    self,
    NavigateToPose,
    'navigate_to_pose',
    self.execute_callback
)

async def execute_callback(self, goal_handle):
    feedback = NavigateToPose.Feedback()
    for i in range(100):
        feedback.progress = i / 100.0
        goal_handle.publish_feedback(feedback)
        await asyncio.sleep(0.1)
    goal_handle.succeed()
    return NavigateToPose.Result()
```

Action features:
- **Cancellation** - Client can cancel in-progress goals
- **Feedback** - Periodic progress updates
- **Result** - Final outcome when complete

## Messages and Interfaces

### Standard Message Types

Common ROS 2 message types:

| Package | Type | Use Case |
|---------|------|----------|
| std_msgs | String, Int32, Float64 | Simple data |
| geometry_msgs | Pose, Twist, Point | Position/velocity |
| sensor_msgs | Image, LaserScan, Imu | Sensor data |
| nav_msgs | Odometry, Path, OccupancyGrid | Navigation |

### Custom Messages

Define custom messages in `.msg` files:

```
# MyMessage.msg
string name
float64[] values
geometry_msgs/Point position
```

## Launch System

Launch files start multiple nodes with configuration:

```python
# launch/robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='robot_controller',
            parameters=[{'speed': 1.0}],
            remappings=[('/cmd_vel', '/robot/cmd_vel')]
        ),
    ])
```

## Parameters

Parameters configure node behavior at runtime:

```python
# Declare parameter
self.declare_parameter('max_speed', 1.0)

# Get parameter
speed = self.get_parameter('max_speed').value

# Set parameter (from command line)
# ros2 param set /node_name max_speed 2.0
```

## TF2: Transform System

TF2 manages coordinate frame transforms:

```python
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# Broadcast transform
t = TransformStamped()
t.header.stamp = self.get_clock().now().to_msg()
t.header.frame_id = 'base_link'
t.child_frame_id = 'camera_link'
t.transform.translation.x = 0.1
self.tf_broadcaster.sendTransform(t)

# Lookup transform
transform = self.tf_buffer.lookup_transform(
    'base_link',
    'camera_link',
    rclpy.time.Time()
)
```

Common frames:
- `map` - Global fixed frame
- `odom` - Odometry frame (drifts over time)
- `base_link` - Robot body frame
- `base_footprint` - Ground projection

## Quality of Service (QoS)

QoS profiles control communication behavior:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)

self.publisher = self.create_publisher(String, 'topic', qos)
```

Key QoS settings:
- **Reliability**: RELIABLE (guaranteed) vs BEST_EFFORT (fast)
- **Durability**: TRANSIENT_LOCAL (late-joining) vs VOLATILE
- **History**: KEEP_LAST vs KEEP_ALL

## Command Line Tools

Essential ROS 2 CLI commands:

```bash
# List active nodes
ros2 node list

# Show node info
ros2 node info /my_node

# List topics
ros2 topic list

# Echo topic data
ros2 topic echo /scan

# Call a service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"

# Record and play bags
ros2 bag record -a
ros2 bag play my_bag
```

## Summary

Key ROS 2 concepts:
- Nodes are modular processes
- Topics for streaming data
- Services for request-response
- Actions for long-running tasks
- TF2 for coordinate transforms
- QoS for communication tuning

---

← **Previous:** [Humanoid Basics](./humanoid-basics.md) | **Next:** [Digital Twin Simulation](./digital-twin.md) →
