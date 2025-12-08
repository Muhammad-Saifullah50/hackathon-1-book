---
id: 0001
title: specify-module-2-digital-twin
stage: spec
date: 2025-12-08
surface: agent
model: gemini-2.0-flash-exp
feature: 004-module-2-digital-twin
branch: 004-module-2-digital-twin
user: saifullah
command: /sp.specify
labels: ["spec", "module-2", "digital-twin", "content"]
links:
  spec: specs/004-module-2-digital-twin/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/004-module-2-digital-twin/spec.md
  - specs/004-module-2-digital-twin/checklists/requirements.md
tests: []
---

## Prompt

# Content Directive: Module 2 (The Digital Twin) ## Context & Source Material * **Source:** Hackathon PDF "Physical AI & Humanoid Robotics". * **Scope:** Weeks 6-7 (Robot Simulation with Gazebo & Unity). * **Goal:** Create a "Digital Twin" to simulate physics, sensors, and environments before deploying to hardware. ## Critical Development Rule (Code & Tech) * **FORCED CONTEXT:** You are strictly forbidden from hallucinating or generating code from your own knowledge base for Gazebo, URDF, or Unity. * **MANDATORY SOURCE:** You must retrieve all technical implementation details, syntax, and API references solely from **`context7`** (Technical Documentation Vault). * **Code Examples:** All code snippets (XML for URDF, C#/Python for Unity/Gazebo plugins) must be derived directly from the official documentation provided in `context7`. ## Chapter 1: The Physics Engine (Gazebo) **File Name:** `docs/module-02/01-gazebo-physics.mdx` ### 1. The Environment (The Matrix) * **Concept:** Introduction to Gazebo as the physics simulator. Explain the difference between "Visual Geometry" (what looks good) and "Collision Geometry" (what physics interacts with). * **Setup:** Guide the user to launch a basic empty world. * **Physics parameters:** Explain gravity, friction, and inertia. * **Sim-to-Real Warning:** Use a specific `<Alert>` component to explain that "perfect friction" in Gazebo does not exist in the real world. ### 2. Robot Description (Advanced URDF & SDF) * **Concept:** Deep dive into URDF (Unified Robot Description Format) and SDF (Simulation Description Format). * **Implementation:** * Define a robot with visual and collision properties. * **Consult `context7`** for the correct XML tag structure for `<inertial>`, `<mass>`, and `<inertia>`. * **Activity:** "The Wobbly Bot." Create a simple robot with incorrect inertia values to show how physics engines explode/fail, then fix it using correct calculations. ### 3. Sensor Simulation * **Concept:** Adding virtual eyes and ears to the digital twin. * **Sensors:** * **LiDAR:** Ray-casting simulation. * **Depth Camera:** Generating depth maps from the render engine. * **Code Lab:** Add a `<sensor>` plugin to the URDF file. * **Directive:** Fetch the exact plugin syntax (e.g., `libgazebo_ros_laser.so`) from **`context7`**. Do not invent plugin names. --- ## Chapter 2: High-Fidelity Rendering (Unity) **File Name:** `docs/module-02/02-unity-integration.mdx` ### 1. Unity for Robotics * **Concept:** Why use Unity? (Better graphics, Human-Robot Interaction, VR interfaces) vs. Gazebo (Pure physics). * **Integration:** How to bridge ROS 2 with Unity (using the ROS-TCP Connector). * **Consult `context7`:** Use the documentation to explain the setup of the "ROS-TCP Endpoint" node. ### 2. The Digital Twin Scene * **Objective:** Build a realistic room environment in Unity where the robot will operate. * **Assets:** Importing the URDF file into Unity. * **Visuals:** Setting up lighting and materials to test computer vision robustness (e.g., shadows, glare). --- ## Module 2 Capstone: The Virtual Obstacle Course **File Name:** `docs/module-02/03-capstone-digital-twin.mdx` ### Assignment * **Goal:** Build a complete simulation environment in Gazebo. * **Requirements:** 1. Create a custom world file (`.world`) with walls and obstacles (cubes/cylinders). 2. Spawn the "Blind Walker" robot from Module 1 into this world. 3. Equip the robot with a simulated LiDAR sensor. * **Assessment Logic:** * The robot must successfully publish "LaserScan" data to a ROS 2 topic. * **Validation:** Verify the data using `ros2 topic echo`. * **Code/Config:** * **STRICTLY** derive the Launch file structure for spawning a robot in Gazebo from **`context7`**.

## Response snapshot

Created feature branch `004-module-2-digital-twin` and specification file defining the content for Module 2: The Digital Twin. Validated with checklist. All requirements from the input were mapped to functional requirements and success criteria. The critical directive regarding mandatory `context7` usage for all technical details and code snippets has been explicitly noted in the spec.

## Outcome

- ✅ Impact: Detailed specification for Module 2 content, emphasizing strict `context7` usage.
- 🧪 Tests: n/a
- 📁 Files: Created spec and checklist for Module 2.
- 🔁 Next prompts: /sp.plan
- 🧠 Reflection: Successfully incorporated the strict `context7` sourcing requirement into the specification.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): none