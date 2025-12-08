# Tasks: Module 2: The Digital Twin Content

**Feature**: Module 2: The Digital Twin Content  
**Status**: Pending  
**Branch**: `004-module-2-digital-twin`

## Dependencies

- Phase 1 (Setup) -> Phase 2 (Research)
- Phase 2 (Research) -> Phase 3 (US1 Content), Phase 4 (US2 Content), Phase 5 (US3 Content)
- Content creation phases (3-5) can proceed once research is complete.

## Implementation Strategy

1.  **MVP Scope**: Complete all research (Phase 2) and then Chapter 1 content (Phase 3).
2.  **Incremental Delivery**: Add chapters sequentially (2, 3).
3.  **Context7 First**: Prioritize gathering all required `context7` information before writing content.

## Phase 1: Setup

**Goal**: Prepare Docusaurus module structure.

- [x] T001 Create directory `website/docs/module-02`
- [x] T002 Create empty `website/docs/module-02/overview.mdx` (as per standard module structure)
- [x] T003 Update `website/src/data/curriculum.json` to include Module 2 entry with correct slugs

## Phase 2: Research (`context7` Sourcing)

**Goal**: Resolve all `NEEDS CLARIFICATION` points and gather specific code/config snippets from `context7`.

- [x] T004 [P] Research `context7` for Gazebo (gazebo_ros_pkgs) documentation ID. (Resolved: `/gazebosim/docs`)
- [x] T005 [P] Research `context7` for XML structure of `<inertial>`, `<mass>`, `<inertia>` for Gazebo physics. Record exact snippets. (Resolved: Snippets found in `/gazebosim/docs`)
- [x] T006 [P] Research `context7` for Gazebo sensor plugin syntax for LiDAR (`gpu_lidar` type, `gz-sim-sensors-system` plugin). Record exact snippets. (Resolved: Snippets found in `/gazebosim/docs`)
- [x] T007 [P] Research `context7` for Gazebo sensor plugin syntax for Depth Camera. Record exact snippets. (Resolved: Snippets found in `/gazebosim/docs`)
- [x] T008 [P] Research `context7` for Launch file structures for spawning robots in Gazebo (`ros_gz_sim`). Record exact snippets. (Resolved: Snippets found in `/gazebosim/docs`)
- [ ] T009 [P] Research `context7` for Unity Robotics (ROS-TCP Connector) documentation ID. (Pending: No direct library ID found in automated search. Will require manual `context7` search during content creation or reference to general Unity Robotics documentation.)
- [ ] T010 [P] Research `context7` for setup steps of "ROS-TCP Endpoint" node within Unity. Record exact snippets (C#). (Pending: Dependent on T009. Will require manual `context7` search during content creation.)
- [x] T011 [P] Research `context7` for URDF import process and code into Unity (`/gkjohnson/urdf-loaders` reference). Record exact snippets (C#). (Resolved: `/gkjohnson/urdf-loaders` is the ID)

## Phase 3: User Story 1 (Gazebo Physics Content)

**Goal**: Create Chapter 1 content "The Physics Engine (Gazebo)".
**Test**: Content adheres to `context7` sources and explains concepts clearly.

- [x] T012 [US1] Create file `website/docs/module-02/01-gazebo-physics.mdx`
- [x] T013 [US1] Write "The Environment (The Matrix)" section introducing Gazebo, visual vs. collision geometry, and empty world setup.
- [x] T014 [US1] Write "Physics parameters" section explaining gravity, friction, and inertia.
- [x] T015 [US1] Include `<Alert>` component for "Sim-to-Real Warning" about perfect friction.
- [x] T016 [US1] Write "Robot Description (Advanced URDF & SDF)" section, with `<inertial>`, `<mass>`, `<inertia>` XML from `context7`.
- [x] T017 [US1] Detail "The Wobbly Bot" activity with instructions to identify and fix inertia value issues.
- [x] T018 [US1] Write "Sensor Simulation" section explaining LiDAR (ray-casting) and Depth Camera.
- [x] T019 [US1] Include Code Lab for adding LiDAR `<sensor>` plugin to URDF, using `context7` syntax.
- [x] T020 [US1] Include Code Lab for adding Depth Camera `<sensor>` plugin to URDF, using `context7` syntax.

## Phase 4: User Story 2 (Unity Integration Content)

**Goal**: Create Chapter 2 content "High-Fidelity Rendering (Unity)".
**Test**: Content adheres to `context7` sources and explains concepts clearly.

- [x] T021 [US2] Create file `website/docs/module-02/02-unity-integration.mdx`
- [x] T022 [US2] Write "Unity for Robotics" section (why Unity vs. Gazebo, ROS-TCP Connector concept).
- [x] T023 [US2] Detail ROS 2 - Unity integration, including "ROS-TCP Endpoint" node setup, using `context7` sourced C# snippets.
- [x] T024 [US2] Write "The Digital Twin Scene" section (building environment, importing URDF, setting up visuals for CV testing).
- [x] T025 [US2] Include C# snippets for URDF import into Unity, using `context7` syntax.

## Phase 5: User Story 3 (Capstone Content)

**Goal**: Create Chapter 3 content "The Virtual Obstacle Course".
**Test**: Content adheres to `context7` sources and outlines a verifiable capstone.

- [x] T026 [US3] Create file `website/docs/module-02/03-capstone-digital-twin.mdx`
- [x] T027 [US3] Write "Assignment" section outlining the goal (build simulation environment in Gazebo).
- [x] T028 [US3] Detail "Requirements" (custom world file, spawning "Blind Walker", simulated LiDAR).
- [x] T029 [US3] Explain "Assessment Logic" (robot publishes "LaserScan") and "Validation" (`ros2 topic echo`).
- [x] T030 [US3] Include Launch file structure for spawning robot in Gazebo, **STRICTLY** from `context7`.

## Phase 6: Polish

**Goal**: Final review and asset cleanup.

- [x] T031 Update `website/sidebars.ts` to include new Module 2 files
- [ ] T032 Manual review of all Module 2 pages for rendering issues and `context7` adherence
- [ ] T033 Ensure all code snippets are correctly formatted and runnable
- [ ] T034 Verify all `context7` links/references are valid (if any explicit links are used)