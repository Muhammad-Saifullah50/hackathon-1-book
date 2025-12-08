# Tasks: Module 1 Content

**Feature**: Module 1: The Robotic Nervous System Content  
**Status**: Pending  
**Branch**: `003-module-1-content`

## Dependencies

- Phase 1 (Setup) -> Phase 2 (Foundation)
- Phase 2 -> Phase 3 (US1), Phase 4 (US2), Phase 5 (US3), Phase 6 (US4)
- US1, US2, US3, US4 can conceptually be written in parallel once Phase 2 is done, but sequential order is safer for consistency.

## Implementation Strategy

1.  **MVP Scope**: Setup components + Chapter 1 Content.
2.  **Incremental Delivery**: Add chapters sequentially (2, 3, 4).
3.  **Parallelism**: Component implementation (Phase 2) can run alongside Phase 1 setup. Content writing (Phases 3-6) can be parallelized if multiple writers, but here sequential.

## Phase 1: Setup

**Goal**: Prepare Docusaurus and dependencies.

- [x] T001 Install `@docusaurus/theme-mermaid` package in `website/`
- [x] T002 Update `website/docusaurus.config.ts` to enable Mermaid theme
- [x] T003 Install `lucide-react`, `clsx`, `tailwind-merge` in `website/` if not present (verification)

## Phase 2: Foundation (Components)

**Goal**: Implement shared UI components required by content standards.

- [x] T004 Create `website/src/components/ui/card.tsx` with Shadcn-like styling
- [x] T005 Create `website/src/components/ui/alert.tsx` with `variant="destructive"` support
- [x] T006 Create `website/src/components/ui/badge.tsx` with variants
- [x] T007 Create `website/src/components/ui/table.tsx` for structured data display
- [x] T008 Create `website/src/components/PersonalizationBar/index.tsx` (Interactive Component)
- [x] T009 Register/Export components for global use in MDX if needed, or document import path

## Phase 3: User Story 1 (Embodied Intelligence)

**Goal**: Create Chapter 1 content "The Awakening".
**Test**: Verify "Three Brains" explanation and interactive Card.

- [x] T010 [US1] Create file `website/docs/module-01/01-embodied-intelligence.mdx`
- [x] T011 [US1] Write "Introduction" and "Key Distinction" sections in `01-embodied-intelligence.mdx`
- [x] T012 [US1] Write "Hardware Reality Check" section with `<Card>` element for cost comparison
- [x] T013 [US1] Write "The Senses" section covering LiDAR, Depth Cameras, IMU, Proprioception
- [x] T014 [US1] Verify citations [cite: XX] are included in Chapter 1

## Phase 4: User Story 2 (ROS 2 Fundamentals)

**Goal**: Create Chapter 2 content "The Robotic Nervous System".
**Test**: Run "Hello Robot" code snippet.

- [x] T015 [US2] Create file `website/docs/module-01/02-ros2-fundamentals.mdx`
- [x] T016 [US2] Write "What is ROS 2?" and "Core Architecture" sections
- [x] T017 [US2] Write "Orchestration" section with Parameters/Launch Files explanation
- [x] T018 [US2] Create "Hello Robot" Code Lab with `<PersonalizationBar>` integration
- [x] T019 [US2] Add `<Tabs>` for Python Agent code and `<Alert>` for Sim-to-Real warning

## Phase 5: User Story 3 (Robot Anatomy)

**Goal**: Create Chapter 3 content "Anatomy of a Humanoid".
**Test**: Visual check of Mermaid URDF diagram.

- [x] T020 [US3] Create file `website/docs/module-01/03-urdf-and-tf.mdx`
- [x] T021 [US3] Write "URDF: The DNA of the Robot" section
- [x] T022 [US3] Add Mermaid.js diagram for URDF Tree Structure
- [x] T023 [US3] Write "The TF Tree" section and "Debug the Ghost Robot" activity

## Phase 6: User Story 4 (Capstone)

**Goal**: Create Chapter 4 content "The Blind Walker".
**Test**: Verify project requirements and logic flow.

- [x] T024 [US4] Create file `website/docs/module-01/04-capstone-project.mdx`
- [x] T025 [US4] Write "Assignment" section with "Brain Node" logic
- [x] T026 [US4] Document "Requirements" (rclpy, launch files, Float32)
- [x] T027 [US4] Add submission/verification instructions (if applicable)

## Phase 7: Polish

**Goal**: Final review and asset cleanup.

- [x] T028 Update `website/sidebars.ts` to include new Module 1 files if not auto-generated
- [x] T029 Manual review of all pages for rendering issues
- [x] T030 Ensure all placeholder images (if any) are valid or replaced with real assets
