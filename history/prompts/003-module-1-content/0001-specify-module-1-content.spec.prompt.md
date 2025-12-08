---
id: 0001
title: specify-module-1-content
stage: spec
date: 2025-12-08
surface: agent
model: gemini-2.0-flash-exp
feature: 003-module-1-content
branch: 003-module-1-content
user: saifullah
command: /sp.specify
labels: ["spec", "module-1", "content"]
links:
  spec: specs/003-module-1-content/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-module-1-content/spec.md
  - specs/003-module-1-content/checklists/requirements.md
tests: []
---

## Prompt

write the contents for module 1 # Content Directive: Module 1 (The Robotic Nervous System) ## Context & Source Material * **Source:** Hackathon PDF "Physical AI & Humanoid Robotics". * **Scope:** Weeks 1-5 (Introduction + ROS 2 Fundamentals). * **Goal:** Bridge the gap between "Digital Brain" and "Physical Body". ## Chapter 1: The Awakening (Weeks 1-2) **File Name:** `docs/module-01/01-embodied-intelligence.mdx` ### 1. Introduction: The Era of Embodied Intelligence * [cite_start]**Concept:** Explain that AI is moving from screens to the physical world[cite: 51]. * **Key Distinction:** "Digital AI" (ChatGPT) vs. "Physical AI" (Robots). * *Physics is the Limit:* Software AI has infinite undo; Physical AI has gravity, friction, and battery life. * *The Loop:* Sense -> Think -> Act. * **Hardware Reality Check (Crucial Section):** * [cite_start]Explain the "Three Brains" architecture[cite: 165]: 1. [cite_start]**The Simulator (God Mode):** High-Performance Workstation (RTX 4070 Ti+)[cite: 119]. 2. [cite_start]**The Edge Brain (Inference):** NVIDIA Jetson Orin Nano/NX[cite: 135]. 3. [cite_start]**The Body:** Unitree Go2 or G1 Humanoid[cite: 149, 160]. * [cite_start]**Interactive Element:** A `<Card>` showing the cost difference between "Cloud Lab" (AWS g5.2xlarge) and "Home Lab"[cite: 173]. ### [cite_start]2. The Senses: How Robots "Feel" [cite: 95] * **Focus:** Sensor systems are the robot's eyes and ears. * **Details:** * **LiDAR:** "Laser Eyes" for mapping (explain Point Clouds). * [cite_start]**Depth Cameras (RealSense):** RGB + D (Color + Distance)[cite: 139]. * [cite_start]**IMU (The Inner Ear):** Accelerometers and Gyroscopes for balance (Crucial for humanoids)[cite: 141]. * **Proprioception:** Force/Torque sensors (feeling the ground). --- ## Chapter 2: The Robotic Nervous System (Weeks 3-5) **File Name:** `docs/module-01/02-ros2-fundamentals.mdx` ### [cite_start]1. What is ROS 2? [cite: 97] * [cite_start]**Analogy:** ROS 2 is not an OS (like Windows); it is the "Nervous System" or Middleware[cite: 57]. * **Why ROS 2?** Real-time capability (DDS), Industry Standard, Python support (`rclpy`). ### [cite_start]2. Core Architecture (The "Graph") * **Nodes:** The neurons. Small, single-purpose programs (e.g., `camera_node`, `motor_node`). * **Topics:** The veins. Unidirectional data streams (Pub/Sub). * *Example:* `camera_node` publishes images to `/camera/raw`. * **Services:** The reflex. Request/Response (e.g., "Reset Odometry"). * **Actions:** The long-term goals. (e.g., "Walk to the kitchen" - takes time, can be cancelled). ### [cite_start]3. Orchestration: Parameters & Launch Files * **Parameters:** Dynamic configuration (e.g., changing max speed without recompiling code). * **Launch Files:** The conductor. Using Python (`.launch.py`) to start multiple nodes (Camera + Motor + Brain) simultaneously. * **Code Example:** Show a simple `LaunchDescription` that sets a `speed_limit` parameter. ### [cite_start]4. The "Hello Robot" Code Lab [cite: 59] * **Objective:** Write a Python Agent using `rclpy` that simulates a heartbeat. * **Personalization Trigger:** * *Software Engineer:* "Focus on the Class structure and Event Loop." * *Hardware Engineer:* "Focus on the frequency (Hz) and real-time constraints." * **Code Snippet:** ```python import rclpy from rclpy.node import Node class HeartbeatNode(Node): def __init__(self): super().__init__('heartbeat') self.publisher_ = self.create_publisher(String, 'status', 10) self.timer = self.create_timer(1.0, self.publish_status) # 1Hz ``` * **Sim-to-Real Warning:** "On your workstation, this runs perfectly. On a robot, other nodes might starve this process if your QoS (Quality of Service) isn't set correctly." --- ## Chapter 3: Anatomy of a Humanoid (URDF) **File Name:** `docs/module-01/03-urdf-and-tf.mdx` ### [cite_start]1. URDF: The DNA of the Robot [cite: 60, 99] * **Definition:** Unified Robot Description Format. * **Structure:** XML file defining Links (bones) and Joints (muscles). * **Visual:** Trigger a [Diagram of URDF Tree Structure] showing `base_link` -> `torso` -> `head`. ### 2. The TF Tree (Transforms) * **Concept:** How the robot knows where its hand is relative to its eye. * **Math:** Brief intro to Translation and Rotation (Quaternions). * **Activity:** "Debug the Ghost Robot." (Common error where TF tree is broken and the robot looks exploded in Rviz). --- ## [cite_start]Module 1 Capstone: The "Blind Walker" [cite: 108] **File Name:** `docs/module-01/04-capstone-project.mdx` ### Assignment * **Goal:** Create a ROS 2 package where a "Brain Node" listens to a mock "IMU Node." * **Logic:** * If IMU detects `tilt > 30 degrees` (Robot is falling), publish a `STOP` command to the "Motor Node." * **Requirements:** * [cite_start]Use `rclpy`[cite: 59]. * [cite_start]Create a custom launch file `walker.launch.py` to start both nodes. * Handle data types correctly (Float32 for sensor data).

## Response snapshot

Created feature branch `003-module-1-content` and specification file defining the content for Module 1: The Robotic Nervous System. Validated with checklist.

## Outcome

- ✅ Impact: Defined detailed content for Module 1, ready for generation.
- 🧪 Tests: n/a
- 📁 Files: Created spec and checklist for Module 1.
- 🔁 Next prompts: /sp.plan
- 🧠 Reflection: Successfully translated detailed content outline into a structured specification.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): none