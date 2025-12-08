# Feature Specification: Module 4: Vision-Language-Action (VLA) Content

**Feature Branch**: `006-module-4-vla`  
**Created**: 2025-12-08  
**Status**: Draft  
**Input**: User description: "# Content Directive: Module 4 (Vision-Language-Action) ## Context & Source Material * **Source:** Hackathon PDF "Physical AI & Humanoid Robotics". * **Scope:** Weeks 11-13 (Humanoid Mechanics, Conversational Robotics, VLA). * **Goal:** The convergence of Generative AI and Robotics. "The Robot that Listens and Thinks." ## Critical Development Rule (Code & Tech) * **FORCED CONTEXT:** You are strictly forbidden from hallucinating code for OpenAI Whisper, LLM API calls, or VLA architectures. * **MANDATORY SOURCE:** You must retrieve all API implementation details and Python client patterns directly from **`context7`** (Technical Documentation Vault). * **Integration Strategy:** Explain clearly how an LLM (running in the cloud or locally) sends JSON commands to a ROS 2 Action Server. ## Chapter 1: The Voice Interface (Ears) **File Name:** `docs/module-04/01-voice-to-action.mdx` ### 1. Hearing the World * **Concept:** From Audio Waveforms to Text. Introduction to OpenAI Whisper (or open-source equivalents like `distil-whisper`). * **The Pipeline:** Microphone -> Audio Buffer -> Whisper Model -> Text String. * **Hardware Reality:** * *Workstation:* Can run "Large" models. * *Jetson:* Must run "Tiny" or "Base" quantized models. * **Action:** Insert a `<Table>` comparing latency vs. accuracy of different model sizes. ### 2. Code Lab: The "Listening Node" * **Objective:** Create a ROS 2 node that captures audio when a wake word is detected. * **Implementation:** * **Consult `context7`** for the Python audio library (e.g., `PyAudio` or `SoundDevice`) integration. * **Output:** Publish the transcribed text to a topic `/human_command` (String). --- ## Chapter 2: Cognitive Planning (The Brain) **File Name:** `docs/module-04/02-cognitive-planning.mdx` ### 1. From Text to JSON Actions * **Concept:** LLMs are bad at controlling motors directly (too slow/hallucinogenic). They are excellent at *planning*. * **The Translator:** Using an LLM to convert "Clean the room" into a sequence of ROS 2 Actions: 1. `Maps_to(kitchen)` 2. `scan_for(trash)` 3. `pick_up(trash)` * **Prompt Engineering:** Show how to write a "System Prompt" that forces the LLM to output strict JSON for the robot. ### 2. The "VLA" Architecture * **Definition:** Vision-Language-Action. * **Visual:** Trigger a mermaid.js diagram showing: `Camera/Mic` --> `VLA Model` --> `Robot Action`. * **Code Lab:** Build a "Planner Node" in Python. * Subscribes to `/human_command`. * Calls the LLM API (Mock or Real). * Publishes to `/robot_goal`. --- ## Chapter 3: Humanoid Mechanics (The Body) **File Name:** `docs/module-04/03-humanoid-mechanics.mdx` ### 1. Bipedal Locomotion * **Concept:** Why walking is harder than rolling. The "Inverted Pendulum" model. * **Balance:** Introduction to ZMP (Zero Moment Point) – simply explained. * **Hardware Focus:** Reference the **Unitree G1** and its kinematic structure. ### 2. Manipulation * **Concept:** Grasping objects. * **Sim-to-Real:** How to use Inverse Kinematics (IK) to move the hand to a specific coordinate. --- ## The Grand Capstone: The Autonomous Humanoid **File Name:** `docs/module-04/04-final-capstone.mdx` ### Assignment: The "Butler Bot" Simulation * **Scenario:** 1. **Environment:** A simulated apartment in Isaac Sim/Gazebo. 2. **Input:** User speaks "Go to the kitchen and find the red apple." 3. **Process:** * `Voice Node`: Transcribes command. * `Planner Node`: Parses "Kitchen" and "Red Apple". * `Nav Node`: Navigates to the Kitchen zone. * `Vision Node`: Identifies the red apple. * **Deliverable:** A video recording of the robot performing this sequence. ### Assessment Rubric * **Functional:** Does the robot reach the target? (50 pts) * **Cognitive:** Did the Planner correctly parse the intent? (25 pts) * **Code Quality:** Are the ROS 2 nodes modular and well-structured? (25 pts)"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Student Learning about Voice Interfaces for Robots (Priority: P1)

A student navigates to the first chapter of Module 4 to learn about integrating voice commands into robotics using technologies like OpenAI Whisper. They will understand the pipeline from audio capture to text transcription and how to build a ROS 2 "Listening Node."

**Why this priority**: This establishes the initial human-robot interaction layer, crucial for conversational robotics.

**Independent Test**: Verify that a student, after reading `docs/module-04/01-voice-to-action.mdx` and following its Code Lab, can:
1.  Explain the audio-to-text transcription pipeline using Whisper.
2.  Implement a ROS 2 node that captures audio and publishes transcribed text to `/human_command`.

**Acceptance Scenarios**:

1. **Given** a student accesses `docs/module-04/01-voice-to-action.mdx`, **When** they read "Hearing the World," **Then** they understand the pipeline from audio waveform to text string via Whisper.
2. **Given** the "Hardware Reality" section, **When** the student reviews it, **Then** they understand the model size considerations for Workstation vs. Jetson.
3. **Given** the `<Table>` comparing model sizes, **When** the student examines it, **Then** they can identify the trade-offs between latency and accuracy.
4. **Given** the "Listening Node" Code Lab, **When** the student implements it, **Then** the node successfully captures audio, transcribes it, and publishes to `/human_command`.

### User Story 2 - Student Learning about Cognitive Planning (LLMs for Robots) (Priority: P1)

A student progresses to the second chapter to understand how Large Language Models (LLMs) can be used for cognitive planning in robotics, translating natural language commands into actionable JSON sequences.

**Why this priority**: This is the core "thinking" part of the AI-Robot Brain, enabling complex task execution from high-level commands.

**Independent Test**: A student, after completing `docs/module-04/02-cognitive-planning.mdx` and its Code Lab, can:
1.  Explain how an LLM can convert natural language into JSON actions for a robot.
2.  Design a "System Prompt" for LLM output.
3.  Implement a ROS 2 "Planner Node" that interfaces with an LLM.

**Acceptance Scenarios**:

1. **Given** a student accesses `docs/module-04/02-cognitive-planning.mdx`, **When** they read "From Text to JSON Actions," **Then** they understand the role of LLMs in planning a sequence of ROS 2 Actions.
2. **Given** the "Prompt Engineering" section, **When** the student reviews it, **Then** they can construct a system prompt to elicit strict JSON output from an LLM.
3. **Given** the "VLA Architecture" section, **When** they examine the Mermaid.js diagram, **Then** they can describe the Vision-Language-Action flow.
4. **Given** the "Planner Node" Code Lab, **When** the student implements it, **Then** the node subscribes to `/human_command`, calls an LLM, and publishes to `/robot_goal`.

### User Story 3 - Student Learning about Humanoid Mechanics (Priority: P2)

A student explores the unique challenges and concepts of humanoid robot mechanics, focusing on locomotion and manipulation.

**Why this priority**: This provides the physical understanding necessary to appreciate the complexity of controlling humanoid robots.

**Independent Test**: A student, after reading `docs/module-04/03-humanoid-mechanics.mdx`, can:
1.  Explain the "Inverted Pendulum" model for bipedal locomotion.
2.  Describe the concept of ZMP for balance.
3.  Understand the basics of Inverse Kinematics (IK) for manipulation.

**Acceptance Scenarios**:

1. **Given** a student accesses `docs/module-04/03-humanoid-mechanics.mdx`, **When** they read "Bipedal Locomotion," **Then** they understand why walking is complex and the "Inverted Pendulum" model.
2. **Given** the explanation of "Balance" and ZMP, **When** the student reviews it, **Then** they can articulate the core idea of Zero Moment Point.
3. **Given** the "Manipulation" section, **When** the student reviews it, **Then** they understand the concept of Inverse Kinematics for controlling robot end-effectors.

### User Story 4 - Student Completing the Grand Capstone: The Autonomous Humanoid (Priority: P1)

A student integrates voice, cognitive planning, and robot control to create an autonomous humanoid simulation capable of understanding and executing complex natural language commands.

**Why this priority**: This capstone validates the convergence of Generative AI and Robotics, representing the culmination of the module's learning.

**Independent Test**: A student successfully implements the "Butler Bot" simulation in Isaac Sim/Gazebo, where the robot correctly processes a spoken command ("Go to the kitchen and find the red apple") and performs the corresponding actions (transcription, planning, navigation, vision, manipulation). The robot's performance is verifiable via video recording and the assessment rubric.

**Acceptance Scenarios**:

1. **Given** a student accesses `docs/module-04/04-final-capstone.mdx`, **When** they read the "Assignment" and "Scenario," **Then** they understand the overall task of creating a "Butler Bot" in a simulated apartment.
2. **Given** the process description (`Voice Node` -> `Planner Node` -> `Nav Node` -> `Vision Node`), **When** the student implements their solution, **Then** each component correctly performs its function as outlined.
3. **Given** the assessment rubric, **When** the student submits their solution, **Then** the robot's functional and cognitive performance, along with code quality, are evaluated according to the criteria.

### Edge Cases

-   What if the LLM API returns malformed JSON or hallucinates an action not supported by the robot? (Assumption: Prompt Engineering mitigates this, but robust error handling in the "Planner Node" is implied).
-   What if the wake word is not detected or mis-transcribed? (Assumption: Content will discuss robustness of speech recognition.)

## Requirements *(mandatory)*

### Functional Requirements

#### General for Module 4 Content
- **FR-001**: Module 4 content MUST cover weeks 11-13, focusing on Humanoid Mechanics, Conversational Robotics, and VLA.
- **FR-002**: Module 4 content MUST aim to achieve the convergence of Generative AI and Robotics ("The Robot that Listens and Thinks").
- **FR-003**: All API implementation details and Python client patterns for OpenAI Whisper, LLM API calls, or VLA architectures MUST be retrieved directly from `context7` (Technical Documentation Vault).
- **FR-004**: Content MUST clearly explain how an LLM (running in the cloud or locally) sends JSON commands to a ROS 2 Action Server.

#### Chapter 1: The Voice Interface (File: `docs/module-04/01-voice-to-action.mdx`)
- **FR-005**: The chapter MUST introduce OpenAI Whisper (or open-source equivalents like `distil-whisper`) for converting audio waveforms to text.
- **FR-006**: The chapter MUST describe the pipeline: Microphone -> Audio Buffer -> Whisper Model -> Text String.
- **FR-007**: The chapter MUST detail hardware reality for Whisper models: "Workstation" for "Large" models, "Jetson" for "Tiny" or "Base" quantized models.
- **FR-008**: The chapter MUST insert a `<Table>` comparing latency vs. accuracy of different model sizes.
- **FR-009**: The chapter MUST include a Code Lab: "The Listening Node", a ROS 2 node that captures audio when a wake word is detected.
- **FR-010**: The Code Lab MUST consult `context7` for Python audio library (e.g., `PyAudio` or `SoundDevice`) integration.
- **FR-011**: The "Listening Node" MUST publish the transcribed text to a topic `/human_command` (String).

#### Chapter 2: Cognitive Planning (File: `docs/module-04/02-cognitive-planning.mdx`)
- **FR-012**: The chapter MUST explain the concept of LLMs for planning (converting natural language to JSON Actions) rather than direct motor control.
- **FR-013**: The chapter MUST demonstrate how an LLM translates a command (e.g., "Clean the room") into a sequence of ROS 2 Actions (`Maps_to`, `scan_for`, `pick_up`).
- **FR-014**: The chapter MUST show how to write a "System Prompt" that forces the LLM to output strict JSON for the robot.
- **FR-015**: The chapter MUST define the "VLA" (Vision-Language-Action) Architecture.
- **FR-016**: The chapter MUST include a Mermaid.js diagram showing: `Camera/Mic` --> `VLA Model` --> `Robot Action`.
- **FR-017**: The chapter MUST include a Code Lab to build a "Planner Node" in Python.
- **FR-018**: The "Planner Node" MUST subscribe to `/human_command`, call the LLM API (Mock or Real), and publish to `/robot_goal`.

#### Chapter 3: Humanoid Mechanics (File: `docs/module-04/03-humanoid-mechanics.mdx`)
- **FR-019**: The chapter MUST explain Bipedal Locomotion and the "Inverted Pendulum" model.
- **FR-020**: The chapter MUST introduce ZMP (Zero Moment Point) for balance, simply explained.
- **FR-021**: The chapter MUST include a Hardware Focus reference to the Unitree G1 and its kinematic structure.
- **FR-022**: The chapter MUST explain Manipulation concepts (Grasping objects).
- **FR-023**: The chapter MUST explain Sim-to-Real aspects of using Inverse Kinematics (IK) to move the hand to a specific coordinate.

#### The Grand Capstone: The Autonomous Humanoid (File: `docs/module-04/04-final-capstone.mdx`)
- **FR-024**: The capstone assignment MUST be "The Butler Bot" Simulation.
- **FR-025**: The capstone scenario MUST include:
    *   **Environment**: Simulated apartment in Isaac Sim/Gazebo.
    *   **Input**: User speaks "Go to the kitchen and find the red apple."
    *   **Process**: `Voice Node` (transcribes), `Planner Node` (parses intent), `Nav Node` (navigates), `Vision Node` (identifies target).
- **FR-026**: The capstone deliverable MUST be a video recording of the robot performing the sequence.
- **FR-027**: The capstone assessment rubric MUST include: Functional (50 pts), Cognitive (25 pts), Code Quality (25 pts).

### Key Entities

- **Module**: A top-level organizational unit for course content (Module 4).
- **Chapter**: An individual learning unit within a module.
- **Voice Interface**: OpenAI Whisper, Python audio libraries.
- **LLM**: Large Language Model for cognitive planning.
- **VLA Architecture**: Vision-Language-Action pipeline.
- **Humanoid Mechanics**: Bipedal locomotion, ZMP, IK.
- **ROS 2**: Topics, Actions.
- **Simulation Environment**: Isaac Sim/Gazebo.
- **Capstone Project**: "Butler Bot" Simulation.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All four specified chapter files (`01-voice-to-action.mdx`, `02-cognitive-planning.mdx`, `03-humanoid-mechanics.mdx`, `04-final-capstone.mdx`) for Module 4 are created and populated with content according to FR-005 to FR-027.
- **SC-002**: All API implementation details and Python client patterns for Whisper, LLM APIs, and VLA architectures are derived directly from `context7` and verifiable.
- **SC-003**: The `<Table>` comparing Whisper model sizes is correctly implemented and populated.
- **SC-004**: The Mermaid.js diagram for VLA architecture is correctly rendered.
- **SC-005**: All code blocks explicitly adhere to hardware labeling ("Workstation" or "Edge").
- **SC-006**: The capstone project rubric is clear and accurately reflects the assignment.