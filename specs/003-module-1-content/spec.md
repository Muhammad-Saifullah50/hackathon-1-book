# Feature Specification: Module 1: The Robotic Nervous System Content

**Feature Branch**: `003-module-1-content`  
**Created**: 2025-12-08  
**Status**: Draft  
**Input**: User description: "write the contents for module 1 # Content Directive: Module 1 (The Robotic Nervous System) ## Context & Source Material * **Source:** Hackathon PDF "Physical AI & Humanoid Robotics". * **Scope:** Weeks 1-5 (Introduction + ROS 2 Fundamentals). * **Goal:** Bridge the gap between "Digital Brain" and "Physical Body". ## Chapter 1: The Awakening (Weeks 1-2) **File Name:** `docs/module-01/01-embodied-intelligence.mdx` ### 1. Introduction: The Era of Embodied Intelligence * [cite_start]**Concept:** Explain that AI is moving from screens to the physical world[cite: 51]. * **Key Distinction:** "Digital AI" (ChatGPT) vs. "Physical AI" (Robots). * *Physics is the Limit:* Software AI has infinite undo; Physical AI has gravity, friction, and battery life. * *The Loop:* Sense -> Think -> Act. * **Hardware Reality Check (Crucial Section):** * [cite_start]Explain the "Three Brains" architecture[cite: 165]: 1. [cite_start]**The Simulator (God Mode):** High-Performance Workstation (RTX 4070 Ti+)[cite: 119]. 2. [cite_start]**The Edge Brain (Inference):** NVIDIA Jetson Orin Nano/NX[cite: 135]. 3. [cite_start]**The Body:** Unitree Go2 or G1 Humanoid[cite: 149, 160]. * [cite_start]**Interactive Element:** A `<Card>` showing the cost difference between "Cloud Lab" (AWS g5.2xlarge) and "Home Lab"[cite: 173]. ### [cite_start]2. The Senses: How Robots "Feel" [cite: 95] * **Focus:** Sensor systems are the robot's eyes and ears. * **Details:** * **LiDAR:** "Laser Eyes" for mapping (explain Point Clouds). * [cite_start]**Depth Cameras (RealSense):** RGB + D (Color + Distance)[cite: 139]. * [cite_start]**IMU (The Inner Ear):** Accelerometers and Gyroscopes for balance (Crucial for humanoids)[cite: 141]. * **Proprioception:** Force/Torque sensors (feeling the ground). --- ## Chapter 2: The Robotic Nervous System (Weeks 3-5) **File Name:** `docs/module-01/02-ros2-fundamentals.mdx` ### [cite_start]1. What is ROS 2? [cite: 97] * [cite_start]**Analogy:** ROS 2 is not an OS (like Windows); it is the "Nervous System" or Middleware[cite: 57]. * **Why ROS 2?** Real-time capability (DDS), Industry Standard, Python support (`rclpy`). ### [cite_start]2. Core Architecture (The "Graph") * **Nodes:** The neurons. Small, single-purpose programs (e.g., `camera_node`, `motor_node`). * **Topics:** The veins. Unidirectional data streams (Pub/Sub). * *Example:* `camera_node` publishes images to `/camera/raw`. * **Services:** The reflex. Request/Response (e.g., "Reset Odometry"). * **Actions:** The long-term goals. (e.g., "Walk to the kitchen" - takes time, can be cancelled). ### [cite_start]3. Orchestration: Parameters & Launch Files * **Parameters:** Dynamic configuration (e.g., changing max speed without recompiling code). * **Launch Files:** The conductor. Using Python (`.launch.py`) to start multiple nodes (Camera + Motor + Brain) simultaneously. * **Code Example:** Show a simple `LaunchDescription` that sets a `speed_limit` parameter. ### [cite_start]4. The "Hello Robot" Code Lab [cite: 59] * **Objective:** Write a Python Agent using `rclpy` that simulates a heartbeat. * **Personalization Trigger:** * *Software Engineer:* "Focus on the Class structure and Event Loop." * *Hardware Engineer:* "Focus on the frequency (Hz) and real-time constraints." * **Code Snippet:** ```python import rclpy from rclpy.node import Node class HeartbeatNode(Node): def __init__(self): super().__init__('heartbeat') self.publisher_ = self.create_publisher(String, 'status', 10) self.timer = self.create_timer(1.0, self.publish_status) # 1Hz ``` * **Sim-to-Real Warning:** "On your workstation, this runs perfectly. On a robot, other nodes might starve this process if your QoS (Quality of Service) isn't set correctly." --- ## Chapter 3: Anatomy of a Humanoid (URDF) **File Name:** `docs/module-01/03-urdf-and-tf.mdx` ### [cite_start]1. URDF: The DNA of the Robot [cite: 60, 99] * **Definition:** Unified Robot Description Format. * **Structure:** XML file defining Links (bones) and Joints (muscles). * **Visual:** Trigger a [Diagram of URDF Tree Structure] showing `base_link` -> `torso` -> `head`. ### 2. The TF Tree (Transforms) * **Concept:** How the robot knows where its hand is relative to its eye. * **Math:** Brief intro to Translation and Rotation (Quaternions). * **Activity:** "Debug the Ghost Robot." (Common error where TF tree is broken and the robot looks exploded in Rviz). --- ## [cite_start]Module 1 Capstone: The "Blind Walker" [cite: 108] **File Name:** `docs/module-01/04-capstone-project.mdx` ### Assignment * **Goal:** Create a ROS 2 package where a "Brain Node" listens to a mock "IMU Node." * **Logic:** * If IMU detects `tilt > 30 degrees` (Robot is falling), publish a `STOP` command to the "Motor Node." * **Requirements:** * [cite_start]Use `rclpy`[cite: 59]. * [cite_start]Create a custom launch file `walker.launch.py` to start both nodes. * Handle data types correctly (Float32 for sensor data)."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Student Learning about Embodied Intelligence (Priority: P1)

A student navigates to the first chapter of Module 1 to learn about embodied intelligence and the fundamental differences between digital and physical AI.

**Why this priority**: This chapter sets the foundational understanding for the entire module and its concepts.

**Independent Test**: Verify that a student, after reading `docs/module-01/01-embodied-intelligence.mdx`, can accurately describe the "Three Brains" architecture and the "Sense -> Think -> Act" loop, and understand the implications of physics on physical AI.

**Acceptance Scenarios**:

1. **Given** a student accesses `docs/module-01/01-embodied-intelligence.mdx`, **When** they read through the content, **Then** they can articulate the core distinction between Digital AI and Physical AI.
2. **Given** the "Hardware Reality Check" section is presented, **When** the student reviews it, **Then** they understand the roles of Workstation, Jetson, and Robot Body within the "Three Brains" architecture.
3. **Given** the interactive `<Card>` element is displayed, **When** the student interacts with it, **Then** they can compare the cost differences between "Cloud Lab" and "Home Lab" setups.

### User Story 2 - Student Learning about ROS 2 Fundamentals (Priority: P1)

A student progresses to the second chapter to grasp the core concepts and architecture of ROS 2, and to perform a basic coding exercise.

**Why this priority**: ROS 2 is central to developing robotic applications, making this chapter crucial for practical understanding.

**Independent Test**: A student, after completing `docs/module-01/02-ros2-fundamentals.mdx` and its code lab, can explain the ROS 2 graph concepts (Nodes, Topics, Services, Actions) and successfully run the "Hello Robot" Python agent.

**Acceptance Scenarios**:

1. **Given** a student accesses `docs/module-01/02-ros2-fundamentals.mdx`, **When** they read about ROS 2, **Then** they understand its role as a "Nervous System" and its key benefits.
2. **Given** the "Core Architecture" section is presented, **When** the student reviews it, **Then** they can identify and describe Nodes, Topics, Services, and Actions with relevant examples.
3. **Given** the "Hello Robot" Code Lab, **When** the student follows the instructions, **Then** they can implement and run the `HeartbeatNode` Python agent and understand the personalization triggers and Sim-to-Real warning.

### User Story 3 - Student Learning about Robot Anatomy with URDF & TF (Priority: P2)

A student learns how robots are structurally defined and how they understand their own spatial relationships using URDF and the TF tree.

**Why this priority**: Understanding robot anatomy is essential for designing, simulating, and debugging physical robots.

**Independent Test**: After reading `docs/module-01/03-urdf-and-tf.mdx`, a student can describe the purpose of URDF and the TF tree, and identify common issues like the "Ghost Robot" scenario.

**Acceptance Scenarios**:

1. **Given** a student accesses `docs/module-01/03-urdf-and-tf.mdx`, **When** they read about URDF, **Then** they understand it as the robot's "DNA" defining links and joints.
2. **Given** the explanation of the TF Tree, **When** the student reviews it, **Then** they understand how it tracks relative positions of robot parts and its mathematical basis.
3. **Given** the "Debug the Ghost Robot" activity, **When** the student considers it, **Then** they can diagnose and conceptualize solutions for TF tree issues.

### User Story 4 - Student Completing the Module 1 Capstone (Priority: P1)

A student applies their knowledge from Module 1 to complete a practical capstone project, integrating ROS 2 concepts.

**Why this priority**: The capstone validates practical application of learned concepts and reinforces problem-solving skills.

**Independent Test**: A student successfully implements the "Blind Walker" project, demonstrating a working ROS 2 package that controls a "Motor Node" based on "IMU Node" data, and verifies the correct use of `rclpy` and a custom launch file.

**Acceptance Scenarios**:

1. **Given** a student accesses `docs/module-01/04-capstone-project.mdx`, **When** they read the assignment, **Then** they understand the goal of creating a "Brain Node" that reacts to "IMU Node" data to issue `STOP` commands.
2. **Given** the project requirements, **When** the student implements the solution, **Then** they correctly use `rclpy`, create `walker.launch.py`, and handle Float32 data types as specified.

### Edge Cases

- **Missing citations:** What if a content section lacks the required `[cite: XX]` markers? (Assumed to be a content quality issue, caught during review).
- **Incorrect file names:** What if content is placed in the wrong `.mdx` file? (Assumed to be a structural validation issue).
- **Misplaced interactive components:** What if `<PersonalizationBar />` or `<Alert>` are not in their mandatory positions? (Assumed to be a structural validation issue).

## Requirements *(mandatory)*

### Functional Requirements

#### General for Module 1 Content
- **FR-001**: The content for Module 1 MUST cover material for Weeks 1-5, specifically "Introduction + ROS 2 Fundamentals".
- **FR-002**: The Module 1 content MUST effectively bridge the conceptual gap between "Digital Brain" and "Physical Body".

#### Chapter 1: The Awakening (File: `docs/module-01/01-embodied-intelligence.mdx`)
- **FR-003**: The chapter MUST introduce the "Era of Embodied Intelligence," explaining AI's transition to the physical world (cite: 51).
- **FR-004**: The chapter MUST clearly distinguish "Digital AI" (e.g., ChatGPT) from "Physical AI" (Robots), emphasizing physics as a limiting factor.
- **FR-005**: The chapter MUST describe the fundamental "Sense -> Think -> Act" loop.
- **FR-006**: The chapter MUST include a "Hardware Reality Check" section detailing the "Three Brains" architecture: Simulator (Workstation, RTX 4070 Ti+), Edge Brain (NVIDIA Jetson Orin Nano/NX, cite: 135), and The Body (Unitree Go2 or G1 Humanoid, cite: 149, 160).
- **FR-007**: The chapter MUST include an interactive `<Card>` element displaying the cost difference between "Cloud Lab" (AWS g5.2xlarge) and "Home Lab" (cite: 173).
- **FR-008**: The chapter MUST explain how robots "Feel" by detailing sensor systems: LiDAR ("Laser Eyes" for mapping/Point Clouds), Depth Cameras (RealSense, RGB+D, cite: 139), IMU (Accelerometers/Gyroscopes for balance, cite: 141), and Proprioception (Force/Torque sensors).

#### Chapter 2: The Robotic Nervous System (File: `docs/module-01/02-ros2-fundamentals.mdx`)
- **FR-009**: The chapter MUST define ROS 2 using the analogy of a "Nervous System" or Middleware (cite: 57, 97).
- **FR-010**: The chapter MUST highlight key reasons for using ROS 2: Real-time capability (DDS), Industry Standard, and Python support (`rclpy`).
- **FR-011**: The chapter MUST explain the Core Architecture (The "Graph"), detailing:
    - **Nodes**: Small, single-purpose programs (e.g., `camera_node`, `motor_node`).
    - **Topics**: Unidirectional data streams (Pub/Sub) with an example (`camera_node` publishes to `/camera/raw`).
    - **Services**: Request/Response mechanism (e.g., "Reset Odometry").
    - **Actions**: Long-term goals (e.g., "Walk to the kitchen" - cancellable).
- **FR-012**: The chapter MUST explain Orchestration through Parameters (dynamic configuration) and Launch Files (using Python `.launch.py` to start multiple nodes).
- **FR-013**: The chapter MUST include a "Hello Robot" Code Lab (cite: 59) where a Python Agent using `rclpy` simulates a heartbeat.
- **FR-014**: The "Hello Robot" Code Lab MUST feature a Personalization Trigger for Software Engineers (focus on Class structure/Event Loop) and Hardware Engineers (focus on frequency/real-time constraints).
- **FR-015**: The "Hello Robot" Code Lab MUST include the provided Python code snippet for `HeartbeatNode`.
- **FR-016**: The "Hello Robot" Code Lab MUST include a Sim-to-Real Warning regarding workstation vs. robot performance and QoS settings.

#### Chapter 3: Anatomy of a Humanoid (File: `docs/module-01/03-urdf-and-tf.mdx`)
- **FR-017**: The chapter MUST define URDF as the "DNA of the Robot" (cite: 60, 99), explaining its XML structure for Links (bones) and Joints (muscles).
- **FR-018**: The chapter MUST include a visual element (Trigger a [Diagram of URDF Tree Structure]) demonstrating `base_link` -> `torso` -> `head` hierarchy.
- **FR-019**: The chapter MUST explain The TF Tree (Transforms), clarifying how the robot understands relative spatial positions and providing a brief introduction to Translation and Rotation (Quaternions).
- **FR-020**: The chapter MUST include an activity titled "Debug the Ghost Robot," explaining the common error where a broken TF tree makes the robot appear exploded in Rviz.

#### Module 1 Capstone: The "Blind Walker" (File: `docs/module-01/04-capstone-project.mdx`)
- **FR-021**: The capstone assignment MUST require students to create a ROS 2 package where a "Brain Node" listens to a mock "IMU Node."
- **FR-022**: The capstone project logic MUST dictate that if the IMU detects `tilt > 30 degrees`, a `STOP` command is published to the "Motor Node."
- **FR-023**: The capstone project requirements MUST specify the use of `rclpy` (cite: 59), creation of a custom launch file `walker.launch.py`, and correct handling of Float32 data types for sensor data.

### Key Entities

- **Module**: A top-level organizational unit for course content (e.g., Module 1).
- **Chapter**: A sub-unit of a module, focusing on specific learning objectives.
- **Content Section**: A distinct logical part within a chapter (e.g., "Introduction," "Hardware Reality Check").
- **Interactive Element**: UI components or activities designed for student engagement.
- **Code Snippet**: Example code provided for understanding or implementation.
- **Capstone Project**: A comprehensive assignment to apply learned skills.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All four specified chapter files (`01-embodied-intelligence.mdx`, `02-ros2-fundamentals.mdx`, `03-urdf-and-tf.mdx`, `04-capstone-project.mdx`) for Module 1 are successfully created and populated with content according to FR-003 to FR-023.
- **SC-002**: Each chapter demonstrates adherence to the general content rules established in `002-book-content-rules/spec.md`, such as the inclusion of Personalization Triggers and Sim-to-Real Warnings where mandated by the specific content.
- **SC-003**: The provided code snippets in `02-ros2-fundamentals.mdx` and the capstone project in `04-capstone-project.mdx` are syntactically correct and can be executed in a simulated ROS 2 environment.
- **SC-004**: Interactive elements (e.g., `<Card>`) are correctly implemented and functional within the Docusaurus framework.
- **SC-005**: All `[cite: XX]` markers in the content accurately reference the "Hackathon PDF 'Physical AI & Humanoid Robotics'" source material.