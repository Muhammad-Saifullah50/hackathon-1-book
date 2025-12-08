---
id: 0001
title: specify-module-4-vla
stage: spec
date: 2025-12-08
surface: agent
model: gemini-2.0-flash-exp
feature: 006-module-4-vla
branch: 006-module-4-vla
user: saifullah
command: /sp.specify
labels: ["spec", "module-4", "vla", "humanoid", "conversational-robotics", "context7"]
links:
  spec: specs/006-module-4-vla/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/006-module-4-vla/spec.md
  - specs/006-module-4-vla/checklists/requirements.md
tests: []
---

## Prompt

# Content Directive: Module 4 (Vision-Language-Action) ## Context & Source Material * **Source:** Hackathon PDF "Physical AI & Humanoid Robotics". * **Scope:** Weeks 11-13 (Humanoid Mechanics, Conversational Robotics, VLA). * **Goal:** The convergence of Generative AI and Robotics. "The Robot that Listens and Thinks." ## Critical Development Rule (Code & Tech) * **FORCED CONTEXT:** You are strictly forbidden from hallucinating code for OpenAI Whisper, LLM API calls, or VLA architectures. * **MANDATORY SOURCE:** You must retrieve all API implementation details and Python client patterns directly from **`context7`**. * **Integration Strategy:** Explain clearly how an LLM (running in the cloud or locally) sends JSON commands to a ROS 2 Action Server. ## Chapter 1: The Voice Interface (Ears) **File Name:** `docs/module-04/01-voice-to-action.mdx` ### 1. Hearing the World * **Concept:** From Audio Waveforms to Text. Introduction to OpenAI Whisper (or open-source equivalents like `distil-whisper`). * **The Pipeline:** Microphone -> Audio Buffer -> Whisper Model -> Text String. * **Hardware Reality:** * *Workstation:* Can run "Large" models. * *Jetson:* Must run "Tiny" or "Base" quantized models. * **Action:** Insert a `<Table>` comparing latency vs. accuracy of different model sizes. ### 2. Code Lab: The "Listening Node" * **Objective:** Create a ROS 2 node that captures audio when a wake word is detected. * **Implementation:** * **Consult `context7`** for the Python audio library (e.g., `PyAudio` or `SoundDevice`) integration. * **Output:** Publish the transcribed text to a topic `/human_command` (String). --- ## Chapter 2: Cognitive Planning (The Brain) **File Name:** `docs/module-04/02-cognitive-planning.mdx` ### 1. From Text to JSON Actions * **Concept:** LLMs are bad at controlling motors directly (too slow/hallucinogenic). They are excellent at *planning*. * **The Translator:** Using an LLM to convert "Clean the room" into a sequence of ROS 2 Actions: 1. `Maps_to(kitchen)` 2. `scan_for(trash)` 3. `pick_up(trash)` * **Prompt Engineering:** Show how to write a "System Prompt" that forces the LLM to output strict JSON for the robot. ### 2. The "VLA" Architecture * **Definition:** Vision-Language-Action. * **Visual:** Trigger a mermaid.js diagram showing: `Camera/Mic` --> `VLA Model` --> `Robot Action`. * **Code Lab:** Build a "Planner Node" in Python. * Subscribes to `/human_command`. * Calls the LLM API (Mock or Real). * Publishes to `/robot_goal`. --- ## Chapter 3: Humanoid Mechanics (The Body) **File Name:** `docs/module-04/03-humanoid-mechanics.mdx` ### 1. Bipedal Locomotion * **Concept:** Why walking is harder than rolling. The "Inverted Pendulum" model. * **Balance:** Introduction to ZMP (Zero Moment Point) – simply explained. * **Hardware Focus:** Reference the **Unitree G1** and its kinematic structure. ### 2. Manipulation * **Concept:** Grasping objects. * **Sim-to-Real:** How to use Inverse Kinematics (IK) to move the hand to a specific coordinate. --- ## The Grand Capstone: The Autonomous Humanoid **File Name:** `docs/module-04/04-final-capstone.mdx` ### Assignment: The "Butler Bot" Simulation * **Scenario:** 1. **Environment:** A simulated apartment in Isaac Sim/Gazebo. 2. **Input:** User speaks "Go to the kitchen and find the red apple." 3. **Process:** * `Voice Node`: Transcribes command. * `Planner Node`: Parses "Kitchen" and "Red Apple". * `Nav Node`: Navigates to the Kitchen zone. * `Vision Node`: Identifies the red apple. * **Deliverable:** A video recording of the robot performing this sequence. ### Assessment Rubric * **Functional:** Does the robot reach the target? (50 pts) * **Cognitive:** Did the Planner correctly parse the intent? (25 pts) * **Code Quality:** Are the ROS 2 nodes modular and well-structured? (25 pts)"

## Response snapshot

Created feature branch `006-module-4-vla` and specification file defining the content for Module 4: Vision-Language-Action (VLA). Validated with checklist. All requirements from the input were mapped to functional requirements and success criteria, with a strong emphasis on `context7` sourcing, hardware reality, and the LLM-ROS 2 Action Server integration strategy.

## Outcome

- ✅ Impact: Detailed specification for Module 4 content, emphasizing strict `context7` usage, hardware considerations, and LLM integration.
- 🧪 Tests: n/a
- 📁 Files: Created spec and checklist for Module 4.
- 🔁 Next prompts: /sp.plan
- 🧠 Reflection: Successfully incorporated complex AI/robotics concepts and strict sourcing rules into the specification.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): none