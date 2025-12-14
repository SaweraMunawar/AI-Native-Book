---
sidebar_position: 3
title: ROS 2 بنیادیات
description: روبوٹ سافٹ ویئر بنانے کے لیے Robot Operating System 2 سیکھیں
---

# ROS 2 بنیادیات

ROS 2 (Robot Operating System 2) روبوٹکس ڈویلپمنٹ کے لیے معیاری مڈل ویئر فریم ورک ہے۔ یہ باب اس کے بنیادی تصورات اور عملی استعمال کا احاطہ کرتا ہے۔

## ROS 2 کیا ہے؟

ROS 2 آپریٹنگ سسٹم نہیں ہے، بلکہ ایک مڈل ویئر فریم ورک ہے جو فراہم کرتا ہے:

- **کمیونیکیشن انفراسٹرکچر** سافٹ ویئر کمپوننٹس کے درمیان
- **ہارڈ ویئر ایبسٹریکشن** سینسرز اور ایکچویٹرز کے لیے
- **ٹول ایکو سسٹم** ویژولائزیشن، ڈیبگنگ، سمیولیشن کے لیے
- **پیکیج مینجمنٹ** کوڈ ری یوز کے لیے

### ROS 1 پر ROS 2 کیوں؟

| فیچر | ROS 1 | ROS 2 |
|---------|-------|-------|
| کمیونیکیشن | کسٹم (TCPROS) | DDS معیار |
| ریئل ٹائم | محدود | سپورٹڈ |
| سیکیورٹی | بلٹ ان نہیں | DDS سیکیورٹی |
| ملٹی روبوٹ | پیچیدہ | نیٹو سپورٹ |
| پلیٹ فارم | صرف Linux | Linux, Windows, macOS |

## بنیادی تصورات

### نوڈز

نوڈ ایک واحد مقصد، ماڈیولر پروسیس ہے:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('نوڈ شروع ہو گیا!')

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

بہترین طریقے:
- ہر منطقی فنکشن کے لیے ایک نوڈ
- نوڈز آزادانہ طور پر ٹیسٹ کیے جا سکیں
- معنی خیز، وضاحتی نام استعمال کریں

### ٹاپکس

ٹاپکس پبلش-سبسکرائب کمیونیکیشن کو ممکن بناتے ہیں:

```python
# پبلشر
self.publisher = self.create_publisher(String, 'topic_name', 10)
self.publisher.publish(String(data='Hello'))

# سبسکرائبر
self.subscription = self.create_subscription(
    String,
    'topic_name',
    self.callback,
    10
)
```

ٹاپک کی خصوصیات:
- **Asynchronous** - پبلشرز سبسکرائبرز کا انتظار نہیں کرتے
- **Many-to-many** - متعدد پبلشرز اور سبسکرائبرز کی اجازت
- **Typed** - پیغامات کا متعین ڈھانچہ ہوتا ہے

### سروسز

سروسز ہم وقت ریکویسٹ-ریسپانس فراہم کرتی ہیں:

```python
# سروس سرور
self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.callback)

def callback(self, request, response):
    response.sum = request.a + request.b
    return response

# سروس کلائنٹ
client = self.create_client(AddTwoInts, 'add_two_ints')
future = client.call_async(request)
```

سروسز کب استعمال کریں:
- جب آگے بڑھنے سے پہلے جواب کی ضرورت ہو
- آپریشن تیز ہو (< 1 سیکنڈ)
- کال کم ہو

### ایکشنز

ایکشنز طویل مدتی کاموں کو فیڈ بیک کے ساتھ ہینڈل کرتے ہیں:

```python
# ایکشن سرور
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

ایکشن کی خصوصیات:
- **منسوخی** - کلائنٹ جاری اہداف منسوخ کر سکتا ہے
- **فیڈ بیک** - وقتاً فوقتاً پیش رفت کی تازہ کاریاں
- **نتیجہ** - مکمل ہونے پر حتمی نتیجہ

## پیغامات اور انٹرفیسز

### معیاری پیغام کی اقسام

عام ROS 2 پیغام کی اقسام:

| پیکیج | قسم | استعمال |
|---------|------|----------|
| std_msgs | String, Int32, Float64 | سادہ ڈیٹا |
| geometry_msgs | Pose, Twist, Point | پوزیشن/رفتار |
| sensor_msgs | Image, LaserScan, Imu | سینسر ڈیٹا |
| nav_msgs | Odometry, Path, OccupancyGrid | نیویگیشن |

### کسٹم پیغامات

`.msg` فائلوں میں کسٹم پیغامات متعین کریں:

```
# MyMessage.msg
string name
float64[] values
geometry_msgs/Point position
```

## لانچ سسٹم

لانچ فائلز متعدد نوڈز کو کنفیگریشن کے ساتھ شروع کرتی ہیں:

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

## پیرامیٹرز

پیرامیٹرز رن ٹائم پر نوڈ رویے کو کنفیگر کرتے ہیں:

```python
# پیرامیٹر ڈیکلیئر کریں
self.declare_parameter('max_speed', 1.0)

# پیرامیٹر حاصل کریں
speed = self.get_parameter('max_speed').value

# پیرامیٹر سیٹ کریں (کمانڈ لائن سے)
# ros2 param set /node_name max_speed 2.0
```

## TF2: ٹرانسفارم سسٹم

TF2 کوآرڈینیٹ فریم ٹرانسفارمز کا انتظام کرتا ہے:

```python
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# ٹرانسفارم براڈکاسٹ کریں
t = TransformStamped()
t.header.stamp = self.get_clock().now().to_msg()
t.header.frame_id = 'base_link'
t.child_frame_id = 'camera_link'
t.transform.translation.x = 0.1
self.tf_broadcaster.sendTransform(t)

# ٹرانسفارم تلاش کریں
transform = self.tf_buffer.lookup_transform(
    'base_link',
    'camera_link',
    rclpy.time.Time()
)
```

عام فریمز:
- `map` - گلوبل فکسڈ فریم
- `odom` - اوڈومیٹری فریم (وقت کے ساتھ ڈرفٹ ہوتا ہے)
- `base_link` - روبوٹ باڈی فریم
- `base_footprint` - زمینی پروجیکشن

## Quality of Service (QoS)

QoS پروفائلز کمیونیکیشن رویے کو کنٹرول کرتے ہیں:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)

self.publisher = self.create_publisher(String, 'topic', qos)
```

اہم QoS سیٹنگز:
- **Reliability**: RELIABLE (گارنٹی شدہ) بمقابلہ BEST_EFFORT (تیز)
- **Durability**: TRANSIENT_LOCAL (دیر سے شامل ہونے والوں کے لیے) بمقابلہ VOLATILE
- **History**: KEEP_LAST بمقابلہ KEEP_ALL

## کمانڈ لائن ٹولز

ضروری ROS 2 CLI کمانڈز:

```bash
# ایکٹو نوڈز کی فہرست
ros2 node list

# نوڈ کی معلومات دکھائیں
ros2 node info /my_node

# ٹاپکس کی فہرست
ros2 topic list

# ٹاپک ڈیٹا ایکو کریں
ros2 topic echo /scan

# سروس کال کریں
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"

# بیگز ریکارڈ اور پلے کریں
ros2 bag record -a
ros2 bag play my_bag
```

## خلاصہ

اہم ROS 2 تصورات:
- نوڈز ماڈیولر پروسیسز ہیں
- ٹاپکس ڈیٹا سٹریمنگ کے لیے
- سروسز ریکویسٹ-ریسپانس کے لیے
- ایکشنز طویل مدتی کاموں کے لیے
- TF2 کوآرڈینیٹ ٹرانسفارمز کے لیے
- QoS کمیونیکیشن ٹیوننگ کے لیے

---

← **پچھلا:** [ہیومنائڈ بنیادیات](./humanoid-basics.md) | **اگلا:** [ڈیجیٹل ٹوئن سمیولیشن](./digital-twin.md) →
