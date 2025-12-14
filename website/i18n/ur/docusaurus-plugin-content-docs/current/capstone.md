---
sidebar_position: 6
title: کیپسٹون پروجیکٹ
description: شروع سے اپنا فزیکل AI سسٹم بنائیں
---

# کیپسٹون پروجیکٹ

آخری باب تک پہنچنے پر مبارک باد! اس کیپسٹون میں، آپ سیکھی ہوئی ہر چیز کو مکمل فزیکل AI سسٹم بنانے میں لاگو کریں گے۔

## پروجیکٹ کا جائزہ

آپ **زبان سے رہنمائی شدہ مینیپولیشن سسٹم** بنائیں گے جو:

1. قدرتی زبان کی ہدایات وصول کرے
2. کیمرہ استعمال کر کے اشیاء کو سمجھے
3. مینیپولیشن ٹاسکس کی منصوبہ بندی اور ایگزیکیوشن کرے
4. ٹاسک کی تکمیل پر فیڈ بیک فراہم کرے

### سسٹم آرکیٹیکچر

```
┌─────────────────────────────────────────────────────────────┐
│                    کیپسٹون سسٹم                              │
└─────────────────────────────────────────────────────────────┘

┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐
│  آواز    │ → │  زبان    │ → │  ویژن    │ → │  روبوٹ   │
│  ان پٹ   │    │  پارسنگ  │    │  سسٹم    │    │  کنٹرول  │
└──────────┘    └──────────┘    └──────────┘    └──────────┘
      │              │              │              │
      ▼              ▼              ▼              ▼
┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐
│ Whisper  │    │   LLM    │    │  YOLO +  │    │ Motion   │
│   ASR    │    │ Planner  │    │  CLIP    │    │ Planner  │
└──────────┘    └──────────┘    └──────────┘    └──────────┘
```

## پروجیکٹ کی ضروریات

### کم از کم قابل عمل پروڈکٹ (MVP)

آپ کے سسٹم کو:

- [ ] مینیپولیشن ٹاسکس کے لیے ٹیکسٹ کمانڈز قبول کرنا ہوگا
- [ ] کمپیوٹر ویژن استعمال کر کے اشیاء کا پتہ لگانا اور ان کی لوکیشن بتانی ہوگی
- [ ] پک اینڈ پلیس کے لیے موشن پلانز تیار کرنا ہوگا
- [ ] Gazebo سمیولیشن میں پلانز ایگزیکیوٹ کرنا ہوگا
- [ ] وضاحت کے ساتھ کامیابی یا ناکامی کی رپورٹ کرنا ہوگا

### اضافی اہداف

اختیاری اضافہ جات:

- [ ] سپیچ ریکگنیشن استعمال کر کے آواز سے ان پٹ
- [ ] ملٹی سٹیپ ٹاسک پلاننگ
- [ ] ایرر ریکوری اور ری پلاننگ
- [ ] حقیقی روبوٹ تعیناتی
- [ ] Isaac Sim فوٹو ریئلسٹک رینڈرنگ

## نفاذ کی گائیڈ

### فیز 1: ماحول کی سیٹ اپ

اپنا ڈویلپمنٹ ماحول بنائیں:

```bash
# ورک سپیس بنائیں
mkdir -p ~/capstone_ws/src
cd ~/capstone_ws

# سٹارٹر کوڈ کلون کریں
git clone https://github.com/physical-ai-textbook/capstone-starter.git src/capstone

# ورک سپیس بلڈ کریں
colcon build
source install/setup.bash
```

### فیز 2: پرسیپشن سسٹم

آبجیکٹ ڈیٹیکشن اور لوکلائزیشن نافذ کریں:

```python
# perception/object_detector.py
import torch
from ultralytics import YOLO

class ObjectDetector:
    def __init__(self, model_path="yolov8n.pt"):
        self.model = YOLO(model_path)

    def detect(self, image):
        """تصویر میں اشیاء کا پتہ لگائیں۔

        واپسی:
            ڈیٹیکشنز کی فہرست شامل:
            - class_name: شے کی کیٹیگری
            - confidence: ڈیٹیکشن سکور
            - bbox: [x1, y1, x2, y2]
        """
        results = self.model(image)
        detections = []

        for r in results:
            for box in r.boxes:
                detections.append({
                    "class_name": self.model.names[int(box.cls)],
                    "confidence": float(box.conf),
                    "bbox": box.xyxy[0].tolist()
                })

        return detections
```

### فیز 3: زبان کی سمجھ

قدرتی زبان کی کمانڈز پارس کریں:

```python
# language/command_parser.py
from groq import Groq

class CommandParser:
    def __init__(self, api_key):
        self.client = Groq(api_key=api_key)

    def parse(self, command: str) -> dict:
        """قدرتی زبان کی کمانڈ کو ساختی ٹاسک میں پارس کریں۔

        مثال:
            "سرخ کپ اٹھاؤ اور میز پر رکھو"
            → {
                "action": "pick_and_place",
                "target": "سرخ کپ",
                "destination": "میز"
              }
        """
        prompt = f"""اس روبوٹ کمانڈ کو ساختی فارمیٹ میں پارس کریں:
        کمانڈ: {command}

        JSON آؤٹ پٹ کریں: action, target, destination (اگر قابل اطلاق)
        """

        response = self.client.chat.completions.create(
            model="llama-3.1-8b-instant",
            messages=[{"role": "user", "content": prompt}],
        )

        return json.loads(response.choices[0].message.content)
```

### فیز 4: موشن پلاننگ

ٹکراؤ سے پاک ٹریجیکٹریز کی منصوبہ بندی کریں:

```python
# planning/motion_planner.py
import moveit_commander

class MotionPlanner:
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")

    def plan_pick(self, target_pose):
        """پک موشن کی منصوبہ بندی کریں۔

        1. پری گرپ پوز پر جائیں
        2. ٹارگٹ کے قریب جائیں
        3. گرپر بند کریں
        4. اٹھائیں
        """
        # پری گرپ اپروچ
        pre_grasp = target_pose.copy()
        pre_grasp.position.z += 0.1

        self.arm.set_pose_target(pre_grasp)
        plan1 = self.arm.plan()

        # فائنل اپروچ
        self.arm.set_pose_target(target_pose)
        plan2 = self.arm.plan()

        return [plan1, plan2]

    def execute(self, plan):
        """موشن پلان ایگزیکیوٹ کریں۔"""
        return self.arm.execute(plan, wait=True)
```

### فیز 5: سسٹم انٹیگریشن

تمام اجزاء کو یکجا کریں:

```python
# main.py
import rclpy
from rclpy.node import Node

class CapstoneSystem(Node):
    def __init__(self):
        super().__init__('capstone_system')

        # اجزاء شروع کریں
        self.detector = ObjectDetector()
        self.parser = CommandParser(os.getenv("GROQ_API_KEY"))
        self.planner = MotionPlanner()

        # ROS انٹرفیسز
        self.command_sub = self.create_subscription(
            String, '/command', self.command_callback, 10
        )
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

    def command_callback(self, msg):
        """آنے والی کمانڈز ہینڈل کریں۔"""
        command = msg.data
        self.get_logger().info(f"موصول: {command}")

        # کمانڈ پارس کریں
        task = self.parser.parse(command)

        # ٹارگٹ آبجیکٹ کا پتہ لگائیں
        detections = self.detector.detect(self.current_image)
        target = self.find_target(detections, task["target"])

        if target is None:
            self.report_failure(f"{task['target']} نہیں ملا")
            return

        # موشن کی منصوبہ بندی
        target_pose = self.compute_grasp_pose(target)
        plan = self.planner.plan_pick(target_pose)

        # ایگزیکیوٹ کریں
        success = self.planner.execute(plan)

        if success:
            self.report_success(task)
        else:
            self.report_failure("موشن ایگزیکیوشن ناکام")
```

## اپنے سسٹم کی جانچ

### یونٹ ٹیسٹس

انفرادی اجزاء ٹیسٹ کریں:

```python
# tests/test_perception.py
def test_object_detection():
    detector = ObjectDetector()
    image = load_test_image("table_with_objects.png")

    detections = detector.detect(image)

    assert len(detections) > 0
    assert any(d["class_name"] == "cup" for d in detections)

# tests/test_language.py
def test_command_parsing():
    parser = CommandParser(api_key="test_key")

    result = parser.parse("نیلا بلاک اٹھاؤ")

    assert result["action"] == "pick"
    assert result["target"] == "نیلا بلاک"
```

### انٹیگریشن ٹیسٹس

سمیولیشن میں اینڈ ٹو اینڈ ٹیسٹ کریں:

```bash
# ٹرمینل 1: سمیولیشن لانچ کریں
ros2 launch capstone simulation.launch.py

# ٹرمینل 2: ٹیسٹس چلائیں
ros2 launch capstone integration_test.launch.py

# متوقع آؤٹ پٹ:
# [TEST] سرخ کپ اٹھاؤ: کامیاب
# [TEST] میز پر رکھو: کامیاب
# [TEST] بلاکس اسٹیک کریں: کامیاب
```

## جانچ کے معیار

آپ کے کیپسٹون کی جانچ ان بنیادوں پر ہوگی:

| معیار | وزن | تفصیل |
|-----------|--------|-------------|
| فعالیت | 40% | کیا ٹاسکس مکمل ہوتے ہیں؟ |
| کوڈ کوالٹی | 20% | صاف، دستاویزی کوڈ |
| آرکیٹیکچر | 20% | ماڈیولر، قابل توسیع ڈیزائن |
| دستاویزات | 10% | واضح README، استعمال گائیڈ |
| تخلیقیت | 10% | نئے فیچرز، پالش |

### گریڈنگ روبرک

**A (90-100)**: MVP مکمل + 2 اضافی اہداف، بہترین کوڈ کوالٹی
**B (80-89)**: MVP مکمل + 1 اضافی ہدف، اچھی کوڈ کوالٹی
**C (70-79)**: MVP مکمل، قابل قبول کوڈ کوالٹی
**D (60-69)**: جزوی MVP، بہتری کی ضرورت
**F (\<60)**: نامکمل یا غیر فعال

## جمع کروانا

اپنا پروجیکٹ ان چیزوں کے ساتھ جمع کروائیں:

1. **سورس کوڈ** - مکمل ہسٹری کے ساتھ Git ریپوزٹری
2. **دستاویزات** - سیٹ اپ اور استعمال کے ساتھ README
3. **ڈیمو ویڈیو** - 3-5 منٹ کا مظاہرہ
4. **رپورٹ** - طریقہ کار اور نتائج کا 2 صفحے کا خلاصہ

## وسائل

### مفید لنکس

- [MoveIt 2 دستاویزات](https://moveit.picknik.ai/)
- [Gazebo ٹیوٹوریلز](https://gazebosim.org/docs)
- [YOLO Ultralytics](https://docs.ultralytics.com/)
- [Groq API ریفرنس](https://console.groq.com/docs)

### مدد حاصل کرنا

- عام مسائل کے لیے کورس فورم چیک کریں
- دفتری اوقات: منگل 3-5 بجے
- ای میل: capstone-help@physical-ai.edu

## نتیجہ

یہ کیپسٹون درسی کتاب کی ہر چیز کو یکجا کرتا ہے:

- **باب 1**: فزیکل AI تصورات
- **باب 2**: روبوٹ کینیمیٹکس اور مینیپولیشن
- **باب 3**: سسٹم انٹیگریشن کے لیے ROS 2
- **باب 4**: ٹیسٹنگ کے لیے Gazebo سمیولیشن
- **باب 5**: زبان سے رہنمائی شدہ کنٹرول کے لیے VLA تصورات

گڈ لک، اور اپنا فزیکل AI سسٹم بنانے کا لطف اٹھائیں!

---

← **پچھلا:** [Vision-Language-Action سسٹمز](./vla-systems.md)

---

## کورس مکمل!

**فزیکل AI اور ہیومنائڈ روبوٹکس** مکمل کرنے پر مبارک باد!

اب آپ کے پاس بنیاد ہے:
- ہیومنائڈ روبوٹس بنانا اور پروگرام کرنا
- ROS 2 ایپلیکیشنز تیار کرنا
- ڈیجیٹل ٹوئن سمیولیشنز بنانا
- VLA پر مبنی کنٹرول سسٹمز نافذ کرنا

**سیکھتے رہیں، بناتے رہیں، اور روبوٹکس کے مستقبل میں خوش آمدید!**
