---
id: 0001
title: specify-module-3-isaac-ai
stage: spec
date: 2025-12-08
surface: agent
model: gemini-2.0-flash-exp
feature: 005-module-3-isaac-ai
branch: 005-module-3-isaac-ai
user: saifullah
command: /sp.specify
labels: ["spec", "module-3", "nvidia-isaac", "isaac-sim", "isaac-ros", "context7"]
links:
  spec: specs/005-module-3-isaac-ai/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-module-3-isaac-ai/spec.md
  - specs/005-module-3-isaac-ai/checklists/requirements.md
tests: []
---

## Prompt

# Content Directive: Module 3 (The AI-Robot Brain) ## Context & Source Material * **Source:** Hackathon PDF "Physical AI & Humanoid Robotics". * **Scope:** Weeks 8-10 (NVIDIA Isaac Platform). * **Goal:** Master advanced perception, simulation, and navigation using the NVIDIA stack. ## Critical Development Rule (Code & Tech) * **FORCED CONTEXT:** You are strictly forbidden from hallucinating code for NVIDIA Isaac Sim, Omniverse, or Isaac ROS. * **MANDATORY SOURCE:** You must retrieve all Python snippets, USD (Universal Scene Description) workflows, and ROS 2 package configurations directly from **`context7`** (Technical Documentation Vault). * **Hardware Constraints:** You must explicitly label every code block as either "Workstation (Isaac Sim)" or "Edge (Jetson Orin)" to avoid confusion. ## Chapter 1: Photorealistic Simulation (Isaac Sim) **File Name:** `docs/module-03/01-isaac-sim-setup.mdx` ### 1. The Omniverse * **Concept:** Introduction to NVIDIA Omniverse and USD (Universal Scene Description). Explain why high-fidelity rendering (Ray Tracing) matters for AI perception (shadows, reflections). * [cite_start]**Hardware Check:** Insert a `<Callout variant="destructive">` stating the absolute requirement of an NVIDIA RTX 4070 Ti (or higher) with 12GB+ VRAM[cite: 119]. * **Setup:** Guide the user to install the Isaac Sim container/app. ### 2. Synthetic Data Generation * **Concept:** Using the simulator to create training data. "Why label images by hand when the simulator knows exactly where the cup is?" * **Implementation:** * **Consult `context7`** for the "Replicator" API or equivalent domain randomization tools. * **Code Lab:** Write a Python script to spawn a "Cube" 100 times in random lighting conditions and save the labeled dataset. --- ## Chapter 2: Hardware-Accelerated Perception (Isaac ROS) **File Name:** `docs/module-03/02-isaac-ros-gems.mdx` ### 1. The Edge Brain (Jetson) * **Concept:** Moving from the Workstation to the Jetson Orin Nano/NX. Explain "Hardware Acceleration" (CUDA/TensorRT) vs. standard CPU processing. * **The "Gem":** Introduce Isaac ROS packages (Gems) like VSLAM and Nvblox. ### 2. Visual SLAM (VSLAM) * **Definition:** Simultaneous Localization and Mapping using cameras. * **Pipeline:** * Input: Stereo Camera (RealSense). * Process: Isaac ROS VSLAM Node. * Output: Robot Pose (Where am I?) + Map (Where are the walls?). * **Configuration:** **Consult `context7`** for the standard launch file configuration for `isaac_ros_visual_slam`. Do not invent parameters. ### 3. Navigation (Nav2) * **Concept:** Path planning for humanoids. * **Integration:** How Isaac ROS VSLAM feeds data into the standard ROS 2 Nav2 stack. * **Activity:** "Point and Click." Set a goal in Rviz and watch the robot plan a path around obstacles. --- ## Module 3 Capstone: The Perception Pipeline **File Name:** `docs/module-03/03-capstone-perception.mdx` ### Assignment * **Goal:** Create a "Follower Robot" simulation in Isaac Sim. * **Scenario:** 1. Load a warehouse environment in Isaac Sim. 2. Spawn a "Target" (e.g., a person or another robot). 3. Implement a perception node that detects the target's position. * **Requirements:** * Use **Isaac Sim** for the environment. * Use **Isaac ROS** components (or simulated equivalents) for processing. * **Validation:** The robot must rotate to face the target as it moves. * **Code/Config:** * **STRICTLY** derive the Python API calls for controlling the Isaac Sim camera and getting ground-truth data from **`context7`**. Keep in mind that the student is building a robot as you teach"

## Response snapshot

Created feature branch `005-module-3-isaac-ai` and specification file defining the content for Module 3: The AI-Robot Brain. Validated with checklist. All requirements from the input were mapped to functional requirements and success criteria, with a strong emphasis on `context7` sourcing and explicit hardware labeling for code blocks.

## Outcome

- ✅ Impact: Detailed specification for Module 3 content, emphasizing strict `context7` usage and hardware constraints.
- 🧪 Tests: n/a
- 📁 Files: Created spec and checklist for Module 3.
- 🔁 Next prompts: /sp.plan
- 🧠 Reflection: Successfully incorporated the strict `context7` sourcing and hardware labeling requirements into the specification.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): none