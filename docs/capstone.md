---
sidebar_position: 6
title: Capstone Project
description: Build your own Physical AI system from scratch
---

# Capstone Project

Congratulations on reaching the final chapter! In this capstone, you'll apply everything you've learned to build a complete Physical AI system.

## Project Overview

You will build a **language-guided manipulation system** that can:

1. Receive natural language instructions
2. Perceive objects using a camera
3. Plan and execute manipulation tasks
4. Provide feedback on task completion

### System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Capstone System                          │
└─────────────────────────────────────────────────────────────┘

┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐
│  Speech  │ → │ Language │ → │  Vision  │ → │  Robot   │
│   Input  │    │ Parsing  │    │  System  │    │ Control  │
└──────────┘    └──────────┘    └──────────┘    └──────────┘
      │              │              │              │
      ▼              ▼              ▼              ▼
┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐
│ Whisper  │    │   LLM    │    │  YOLO +  │    │ Motion   │
│   ASR    │    │ Planner  │    │  CLIP    │    │ Planner  │
└──────────┘    └──────────┘    └──────────┘    └──────────┘
```

## Project Requirements

### Minimum Viable Product (MVP)

Your system must:

- [ ] Accept text commands for manipulation tasks
- [ ] Detect and localize objects using computer vision
- [ ] Generate motion plans for pick-and-place
- [ ] Execute plans in Gazebo simulation
- [ ] Report success or failure with explanation

### Stretch Goals

Optional enhancements:

- [ ] Voice input using speech recognition
- [ ] Multi-step task planning
- [ ] Error recovery and replanning
- [ ] Real robot deployment
- [ ] Isaac Sim photorealistic rendering

## Implementation Guide

### Phase 1: Environment Setup

Create your development environment:

```bash
# Create workspace
mkdir -p ~/capstone_ws/src
cd ~/capstone_ws

# Clone starter code
git clone https://github.com/physical-ai-textbook/capstone-starter.git src/capstone

# Build workspace
colcon build
source install/setup.bash
```

### Phase 2: Perception System

Implement object detection and localization:

```python
# perception/object_detector.py
import torch
from ultralytics import YOLO

class ObjectDetector:
    def __init__(self, model_path="yolov8n.pt"):
        self.model = YOLO(model_path)

    def detect(self, image):
        """Detect objects in image.

        Returns:
            List of detections with:
            - class_name: Object category
            - confidence: Detection score
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

### Phase 3: Language Understanding

Parse natural language commands:

```python
# language/command_parser.py
from groq import Groq

class CommandParser:
    def __init__(self, api_key):
        self.client = Groq(api_key=api_key)

    def parse(self, command: str) -> dict:
        """Parse natural language command into structured task.

        Example:
            "Pick up the red cup and put it on the table"
            → {
                "action": "pick_and_place",
                "target": "red cup",
                "destination": "table"
              }
        """
        prompt = f"""Parse this robot command into structured format:
        Command: {command}

        Output JSON with: action, target, destination (if applicable)
        """

        response = self.client.chat.completions.create(
            model="llama-3.1-8b-instant",
            messages=[{"role": "user", "content": prompt}],
        )

        return json.loads(response.choices[0].message.content)
```

### Phase 4: Motion Planning

Plan collision-free trajectories:

```python
# planning/motion_planner.py
import moveit_commander

class MotionPlanner:
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")

    def plan_pick(self, target_pose):
        """Plan pick motion.

        1. Move to pre-grasp pose
        2. Approach target
        3. Close gripper
        4. Lift
        """
        # Pre-grasp approach
        pre_grasp = target_pose.copy()
        pre_grasp.position.z += 0.1

        self.arm.set_pose_target(pre_grasp)
        plan1 = self.arm.plan()

        # Final approach
        self.arm.set_pose_target(target_pose)
        plan2 = self.arm.plan()

        return [plan1, plan2]

    def execute(self, plan):
        """Execute motion plan."""
        return self.arm.execute(plan, wait=True)
```

### Phase 5: System Integration

Combine all components:

```python
# main.py
import rclpy
from rclpy.node import Node

class CapstoneSystem(Node):
    def __init__(self):
        super().__init__('capstone_system')

        # Initialize components
        self.detector = ObjectDetector()
        self.parser = CommandParser(os.getenv("GROQ_API_KEY"))
        self.planner = MotionPlanner()

        # ROS interfaces
        self.command_sub = self.create_subscription(
            String, '/command', self.command_callback, 10
        )
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

    def command_callback(self, msg):
        """Handle incoming commands."""
        command = msg.data
        self.get_logger().info(f"Received: {command}")

        # Parse command
        task = self.parser.parse(command)

        # Detect target object
        detections = self.detector.detect(self.current_image)
        target = self.find_target(detections, task["target"])

        if target is None:
            self.report_failure(f"Could not find {task['target']}")
            return

        # Plan motion
        target_pose = self.compute_grasp_pose(target)
        plan = self.planner.plan_pick(target_pose)

        # Execute
        success = self.planner.execute(plan)

        if success:
            self.report_success(task)
        else:
            self.report_failure("Motion execution failed")
```

## Testing Your System

### Unit Tests

Test individual components:

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

    result = parser.parse("Pick up the blue block")

    assert result["action"] == "pick"
    assert result["target"] == "blue block"
```

### Integration Tests

Test end-to-end in simulation:

```bash
# Terminal 1: Launch simulation
ros2 launch capstone simulation.launch.py

# Terminal 2: Run tests
ros2 launch capstone integration_test.launch.py

# Expected output:
# [TEST] Pick red cup: PASSED
# [TEST] Place on table: PASSED
# [TEST] Stack blocks: PASSED
```

## Evaluation Criteria

Your capstone will be evaluated on:

| Criterion | Weight | Description |
|-----------|--------|-------------|
| Functionality | 40% | Does it complete tasks? |
| Code Quality | 20% | Clean, documented code |
| Architecture | 20% | Modular, extensible design |
| Documentation | 10% | Clear README, usage guide |
| Creativity | 10% | Novel features, polish |

### Grading Rubric

**A (90-100)**: MVP complete + 2 stretch goals, excellent code quality
**B (80-89)**: MVP complete + 1 stretch goal, good code quality
**C (70-79)**: MVP complete, acceptable code quality
**D (60-69)**: Partial MVP, needs improvement
**F (\<60)**: Incomplete or non-functional

## Submission

Submit your project with:

1. **Source code** - Git repository with full history
2. **Documentation** - README with setup and usage
3. **Demo video** - 3-5 minute demonstration
4. **Report** - 2-page summary of approach and results

## Resources

### Helpful Links

- [MoveIt 2 Documentation](https://moveit.picknik.ai/)
- [Gazebo Tutorials](https://gazebosim.org/docs)
- [YOLO Ultralytics](https://docs.ultralytics.com/)
- [Groq API Reference](https://console.groq.com/docs)

### Getting Help

- Check the course forum for common issues
- Office hours: Tuesdays 3-5 PM
- Email: capstone-help@physical-ai.edu

## Conclusion

This capstone integrates everything from the textbook:

- **Chapter 1**: Physical AI concepts
- **Chapter 2**: Robot kinematics and manipulation
- **Chapter 3**: ROS 2 for system integration
- **Chapter 4**: Gazebo simulation for testing
- **Chapter 5**: VLA concepts for language-guided control

Good luck, and enjoy building your Physical AI system!

---

← **Previous:** [Vision-Language-Action Systems](./vla-systems.md)

---

## Course Complete!

Congratulations on completing **Physical AI & Humanoid Robotics**!

You now have the foundation to:
- Build and program humanoid robots
- Develop ROS 2 applications
- Create digital twin simulations
- Implement VLA-based control systems

**Keep learning, keep building, and welcome to the future of robotics!**
