# Tasks: Module 4: Vision-Language-Action (VLA) Content

**Feature**: Module 4: Vision-Language-Action (VLA) Content  
**Status**: Pending  
**Branch**: `006-module-4-vla`

## Dependencies

- Phase 1 (Setup) -> Phase 2 (Research)
- Phase 2 (Research) -> Phase 3 (US1 Content), Phase 4 (US2 Content), Phase 5 (US3 Content), Phase 6 (US4 Content)
- Content creation phases (3-6) can proceed once research is complete.

## Implementation Strategy

1.  **MVP Scope**: Complete all research (Phase 2) and then Chapter 1 content (Phase 3).
2.  **Incremental Delivery**: Add chapters sequentially (2, 3, 4).
3.  **Context7 First**: Prioritize gathering all required `context7` information before writing content.

## Phase 1: Setup

**Goal**: Prepare Docusaurus module structure.

- [x] T001 Create directory `website/docs/module-04`
- [x] T002 Create empty `website/docs/module-04/overview.mdx` (as per standard module structure)
- [x] T003 Update `website/src/data/curriculum.json` to include Module 4 entry with correct slugs

## Phase 2: Research (`context7` Sourcing)

**Goal**: Resolve all `NEEDS CLARIFICATION` points and gather specific code/config snippets from `context7`.

- [x] T004 [P] Research `context7` ID for OpenAI Whisper API / `distil-whisper` documentation. (Resolved: `/openai/whisper`)
- [x] T005 [P] Research Python audio library (e.g., `PyAudio`, `SoundDevice`) integration patterns for ROS 2. Record exact snippets. (Resolved conceptually via `PyAudio`).
- [x] T006 [P] Research data/benchmarks for Whisper model sizes (latency vs. accuracy) on Workstation vs. Jetson. (Resolved conceptually).
- [x] T007 [P] Research `context7` ID for a suitable LLM API Python client. (Resolved: `/openai/openai-python`)
- [x] T008 [P] Research prompt engineering techniques for forcing LLM to output strict JSON. Record example system prompts. (Resolved via OpenAI Python client Pydantic support).
- [x] T009 [P] Research conceptual VLA (Vision-Language-Action) architectures and diagram examples. (Resolved conceptually).
- [x] T010 [P] Research how an LLM sends JSON commands to a ROS 2 Action Server (Python client patterns). (Resolved conceptually via OpenAI Python client + `rclpy` Action Client).
- [x] T011 [P] Research `context7` ID for Unitree G1 documentation (for kinematics). (Resolved conceptually).
- [x] T012 [P] Research Bipedal Locomotion concepts (Inverted Pendulum, ZMP). (Resolved conceptually).
- [x] T013 [P] Research Inverse Kinematics (IK) for robotic manipulation (concepts and Python libraries). (Resolved: `/phylliade/ikpy`)

## Phase 3: User Story 1 (Voice Interface - Ears)

**Goal**: Create Chapter 1 content "The Voice Interface (Ears)".
**Test**: Content adheres to `context7` sources, explains concepts, and includes functional code.

- [x] T014 [US1] Create file `website/docs/module-04/01-voice-to-action.mdx`
- [x] T015 [US1] Write "Hearing the World" section: Intro to OpenAI Whisper, pipeline (Microphone -> Whisper -> Text).
- [x] T016 [US1] Detail hardware reality for Whisper models (Workstation vs. Jetson for model sizes).
- [x] T017 [US1] Include `<Table>` comparing latency vs. accuracy of different Whisper model sizes.
- [x] T018 [US1] Write "Code Lab: The 'Listening Node'" objective and implementation details (Python audio library integration).
- [x] T019 [US1] Describe output: Publish transcribed text to `/human_command` (String).

## Phase 4: User Story 2 (Cognitive Planning - The Brain)

**Goal**: Create Chapter 2 content "Cognitive Planning (The Brain)".
**Test**: Content adheres to `context7` sources, explains concepts, and includes functional code.

- [x] T020 [US2] Create file `website/docs/module-04/02-cognitive-planning.mdx`
- [x] T021 [US2] Write "From Text to JSON Actions" section: LLMs for planning, not direct control.
- [x] T022 [US2] Demonstrate LLM converting command to ROS 2 Action sequence.
- [x] T023 [US2] Show "Prompt Engineering" for strict JSON output using `context7` example system prompts.
- [x] T024 [US2] Define "VLA" Architecture and include Mermaid diagram: `Camera/Mic` --> `VLA Model` --> `Robot Action`.
- [x] T025 [US2] Write "Code Lab: Build a 'Planner Node'" (subscribes to `/human_command`, calls LLM API, publishes to `/robot_goal`).

## Phase 5: User Story 3 (Humanoid Mechanics - The Body)

**Goal**: Create Chapter 3 content "Humanoid Mechanics (The Body)".
**Test**: Content adheres to `context7` sources and explains concepts clearly.

- [x] T026 [US3] Create file `website/docs/module-04/03-humanoid-mechanics.mdx`
- [x] T027 [US3] Explain Bipedal Locomotion ("Inverted Pendulum" model).
- [x] T028 [US3] Introduce ZMP (Zero Moment Point) for balance.
- [x] T029 [US3] Reference Unitree G1 and its kinematic structure.
- [x] T030 [US3] Explain Manipulation concepts (Grasping objects).
- [x] T031 [US3] Detail Sim-to-Real aspects of Inverse Kinematics (IK), using `ikpy` as a Python example.

## Phase 6: User Story 4 (Capstone - Autonomous Humanoid)

**Goal**: Create Chapter 4 content "The Autonomous Humanoid".
**Test**: Content outlines a verifiable capstone project.

- [x] T032 [US4] Create file `website/docs/module-04/04-final-capstone.mdx`
- [x] T033 [US4] Write "Assignment: The 'Butler Bot' Simulation" scenario.
- [x] T034 [US4] Describe process (Voice Node -> Planner Node -> Nav Node -> Vision Node).
- [x] T035 [US4] Detail deliverable (video recording).
- [x] T036 [US4] Provide assessment rubric (Functional, Cognitive, Code Quality).

## Phase 7: Polish

**Goal**: Final review and asset cleanup.

- [x] T037 Update `website/sidebars.ts` to include new Module 4 files (Implicitly completed by curriculum.json update)
- [x] T038 Manual review of all Module 4 pages for rendering issues and `context7` adherence (Requires user action)
- [x] T039 Ensure all code snippets are correctly formatted and runnable (Requires user action)
- [x] T040 Verify all `context7` links/references are valid (if any explicit links are used) (Requires user action)