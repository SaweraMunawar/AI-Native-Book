---
sidebar_position: 4
title: ڈیجیٹل ٹوئن سمیولیشن
description: Gazebo اور NVIDIA Isaac Sim کے ساتھ ورچوئل روبوٹ بنائیں
---

# ڈیجیٹل ٹوئن سمیولیشن

ڈیجیٹل ٹوئنز فزیکل روبوٹس کی ورچوئل نقلیں ہیں، جو محفوظ ڈویلپمنٹ، ٹیسٹنگ، اور ٹریننگ کو ممکن بناتی ہیں۔ یہ باب Gazebo اور NVIDIA Isaac Sim کے ساتھ سمیولیشن کا احاطہ کرتا ہے۔

## سمیولیشن کیوں؟

سمیولیشن اہم فوائد فراہم کرتی ہے:

1. **حفاظت** - خطرناک منظرناموں کو بغیر خطرے کے ٹیسٹ کریں
2. **رفتار** - ریئل ٹائم سے تیز چلائیں
3. **لاگت** - کوئی ہارڈ ویئر خراب یا نقصان نہیں
4. **تکرار پذیری** - عین مطابق حالات دہرائیں
5. **اسکیل ایبلٹی** - ہزاروں متوازی مثالیں چلائیں

### سمیولیشن ریئلٹی گیپ

چیلنج: سمیولیٹڈ رویہ حقیقت سے بالکل مماثل نہیں ہوتا۔

| عنصر | سمیولیشن | حقیقت |
|--------|------------|---------|
| فزکس | تخمینی | پیچیدہ |
| سینسرز | صاف ڈیٹا | شور والا |
| ایکچویٹرز | فوری ردعمل | تاخیر، رگڑ |
| ماحول | آسان | غیر متوقع |

پُل کرنے کی تکنیکیں:
- **ڈومین رینڈمائزیشن** - سمیولیشن پیرامیٹرز میں تنوع
- **سسٹم شناخت** - سم کو حقیقی پیمائشوں سے ملانا
- **سم ٹو ریئل ٹرانسفر** - سم میں ٹریننگ، ہارڈ ویئر پر فائن ٹیون

## Gazebo سمیولیشن

Gazebo سب سے زیادہ استعمال ہونے والا اوپن سورس روبوٹ سمیولیٹر ہے۔

### Gazebo آرکیٹیکچر

```
┌─────────────────────────────────────────┐
│              Gazebo سرور                │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  │
│  │ فزکس   │  │ سینسر   │  │ پلگ انز │  │
│  │ انجن    │  │  سم     │  │         │  │
│  └─────────┘  └─────────┘  └─────────┘  │
└─────────────────────────────────────────┘
              ↕ ٹرانسپورٹ لیئر
┌─────────────────────────────────────────┐
│              Gazebo کلائنٹ              │
│  ┌─────────────────────────────────┐    │
│  │      3D ویژولائزیشن GUI        │    │
│  └─────────────────────────────────┘    │
└─────────────────────────────────────────┘
```

### URDF: روبوٹ ڈسکرپشن

URDF (Unified Robot Description Format) روبوٹ ڈھانچے کی وضاحت کرتا ہے:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- بیس لنک -->
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

  <!-- ریوولیوٹ جوڑ -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
</robot>
```

### SDF: سمیولیشن ڈسکرپشن

SDF سمیولیشن-مخصوص فیچرز کے ساتھ URDF کو وسعت دیتا ہے:

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
      <!-- روبوٹ کی تعریف -->
    </model>
  </world>
</sdf>
```

### Gazebo پلگ انز

پلگ انز Gazebo کی فعالیت کو وسعت دیتے ہیں:

```xml
<gazebo>
  <!-- ڈیفرنشیل ڈرائیو کنٹرولر -->
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

عام پلگ انز:
- `gazebo_ros_camera` - کیمرہ سینسر
- `gazebo_ros_imu_sensor` - IMU سینسر
- `gazebo_ros_lidar` - LiDAR سینسر
- `gazebo_ros_joint_state_publisher` - جوڑوں کی حالت

## NVIDIA Isaac Sim

Isaac Sim GPU-ایکسلریٹڈ، فوٹو ریئلسٹک سمیولیشن فراہم کرتا ہے۔

### Isaac Sim فیچرز

| فیچر | فائدہ |
|---------|---------|
| RTX رینڈرنگ | فوٹو ریئلسٹک ویژولز |
| PhysX 5 | جدید فزکس سمیولیشن |
| سنتھیٹک ڈیٹا | خودکار گراؤنڈ ٹروتھ لیبلز |
| ROS 2 برج | براہ راست ROS 2 انٹیگریشن |
| Python API | اسکرپٹ ایبل ورک فلوز |
| کلاؤڈ ڈیپلائمنٹ | اسکیل ایبل ٹریننگ |

### Isaac Sim آرکیٹیکچر

```
┌─────────────────────────────────────────┐
│         NVIDIA Isaac Sim                │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  │
│  │ Omni-   │  │ PhysX   │  │  RTX    │  │
│  │ verse   │  │ فزکس    │  │ رینڈر   │  │
│  └─────────┘  └─────────┘  └─────────┘  │
└─────────────────────────────────────────┘
              ↕ OmniGraph
┌─────────────────────────────────────────┐
│           Isaac ایکسٹینشنز              │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  │
│  │  ROS 2  │  │سنتھیٹک │  │ روبوٹ   │  │
│  │  برج    │  │  ڈیٹا   │  │ ماڈلز   │  │
│  └─────────┘  └─────────┘  └─────────┘  │
└─────────────────────────────────────────┘
```

### Python اسکرپٹنگ

Isaac Sim کو پروگرامیٹک طور پر کنٹرول کریں:

```python
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage

# ورلڈ بنائیں
world = World()

# روبوٹ شامل کریں
add_reference_to_stage(
    usd_path="/path/to/robot.usd",
    prim_path="/World/Robot"
)
robot = world.scene.add(Robot(prim_path="/World/Robot", name="my_robot"))

# سمیولیشن لوپ
world.reset()
while simulation_app.is_running():
    world.step(render=True)
```

### سنتھیٹک ڈیٹا جنریشن

Isaac Sim خودکار طور پر ٹریننگ ڈیٹا تیار کرتا ہے:

- **RGB تصاویر** کیمرہ انٹرنسکس کے ساتھ
- **گہرائی کے نقشے** 3D ادراک کے لیے
- **سیمینٹک سیگمینٹیشن** لیبلز
- **انسٹینس سیگمینٹیشن** آبجیکٹ ڈیٹیکشن کے لیے
- **2D/3D باؤنڈنگ باکسز**
- **سکیلیٹن ڈیٹا** پوز ایسٹیمیشن کے لیے

## فزکس انجنز

دونوں سمیولیٹرز فزکس انجن کے انتخاب پیش کرتے ہیں:

### Gazebo فزکس انجنز

- **ODE** - Open Dynamics Engine (ڈیفالٹ)
- **Bullet** - مقبول گیم فزکس
- **DART** - آرٹیکولیٹڈ روبوٹس کے لیے درست
- **Simbody** - بائیو میکینکس فوکس

### Isaac Sim PhysX 5

PhysX 5 فیچرز:
- GPU ایکسلریشن
- سافٹ باڈی سمیولیشن
- فلوئڈ ڈائنامکس
- پارٹیکل سسٹمز
- ڈیفارمیبل آبجیکٹس

## سینسر سمیولیشن

### کیمرہ سینسرز

```xml
<!-- Gazebo کیمرہ -->
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

### LiDAR سینسرز

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

## خلاصہ

اہم سمیولیشن تصورات:
- ڈیجیٹل ٹوئنز محفوظ، تیز ڈویلپمنٹ کو ممکن بناتے ہیں
- Gazebo اوپن سورس اور وسیع پیمانے پر استعمال ہوتا ہے
- Isaac Sim فوٹو ریئلسٹک GPU سمیولیشن پیش کرتا ہے
- URDF/SDF روبوٹ اور ورلڈ ماڈلز بیان کرتے ہیں
- پلگ انز سینسرز اور کنٹرولرز شامل کرتے ہیں
- سم ٹو ریئل گیپ کو پُر کرنا ضروری ہے

---

← **پچھلا:** [ROS 2 بنیادیات](./ros2-fundamentals.md) | **اگلا:** [Vision-Language-Action سسٹمز](./vla-systems.md) →
