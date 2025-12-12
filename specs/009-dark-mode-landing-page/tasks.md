# Tasks: Dark Mode Landing Page

**Input**: Design documents from `/specs/009-dark-mode-landing-page/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md, quickstart.md

**Tests**: Tests included per TDD mandate in constitution.md
**Organization**: Tasks grouped by user story for independent implementation and testing

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1-US6)
- Include exact file paths in descriptions

## Path Conventions

- **Project root**: `website/`
- **Components**: `website/src/components/landing/`
- **Pages**: `website/src/pages/`
- **Tests**: `website/tests/components/landing/`
- **Assets**: `website/static/img/landing/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization, dependencies, and basic structure

- [ ] T001 Install framer-motion dependency in website/package.json
- [ ] T002 Create landing components directory at website/src/components/landing/
- [ ] T003 [P] Create landing assets directory at website/static/img/landing/
- [ ] T004 [P] Create landing tests directory at website/tests/components/landing/
- [ ] T005 Add placeholder robot image at website/static/img/landing/robot-hero.svg

**Checkpoint**: Project structure ready for component development

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core reusable components that ALL user stories depend on

**CRITICAL**: No user story work can begin until this phase is complete

- [ ] T006 Create AnimatedSection wrapper component at website/src/components/landing/AnimatedSection.tsx
- [ ] T007 [P] Create GlowButton component (primary/secondary variants) at website/src/components/landing/GlowButton.tsx
- [ ] T008 [P] Write test for AnimatedSection at website/tests/components/landing/AnimatedSection.test.tsx
- [ ] T009 [P] Write test for GlowButton at website/tests/components/landing/GlowButton.test.tsx
- [ ] T010 Run tests and verify AnimatedSection and GlowButton tests pass

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - First Impression & Navigation (Priority: P1) MVP

**Goal**: Hero section with "Wake Up the Metal" headline, gradient text, robot visual, and two CTA buttons

**Independent Test**: Load homepage and verify Hero section displays correctly with headline, subheadline, robot visual, and both CTA buttons functional

### Tests for User Story 1

- [ ] T011 [P] [US1] Write test for HeroSection at website/tests/components/landing/HeroSection.test.tsx

### Implementation for User Story 1

- [ ] T012 [US1] Create HeroSection component at website/src/components/landing/HeroSection.tsx
- [ ] T013 [US1] Implement gradient text effect on "Metal" with cyan-to-violet gradient
- [ ] T014 [US1] Add robot visual with fallback placeholder in HeroSection
- [ ] T015 [US1] Add "Start Reading" primary CTA linking to /docs/module-01/overview
- [ ] T016 [US1] Add "Watch Demo" secondary CTA with outline style
- [ ] T017 [US1] Make HeroSection responsive (centered mobile, split desktop)
- [ ] T018 [US1] Run HeroSection test and verify it passes

**Checkpoint**: Hero section fully functional and testable independently

---

## Phase 4: User Story 2 - Understanding the Curriculum (Priority: P1)

**Goal**: Four glassmorphism curriculum cards with icons showing learning journey

**Independent Test**: Scroll to Curriculum section and verify all four cards display with correct content, glassmorphism styling, and icons

### Tests for User Story 2

- [ ] T019 [P] [US2] Write test for CurriculumCard at website/tests/components/landing/CurriculumCard.test.tsx
- [ ] T020 [P] [US2] Write test for CurriculumSection at website/tests/components/landing/CurriculumSection.test.tsx

### Implementation for User Story 2

- [ ] T021 [P] [US2] Create CurriculumCard component with glassmorphism styling at website/src/components/landing/CurriculumCard.tsx
- [ ] T022 [US2] Create CurriculumSection with title "The Journey to Embodied Intelligence" at website/src/components/landing/CurriculumSection.tsx
- [ ] T023 [US2] Add curriculum data array with 4 items (Network, Box, BrainCircuit, Bot icons) in CurriculumSection
- [ ] T024 [US2] Implement responsive grid layout (4-col desktop, 1-col mobile) in CurriculumSection
- [ ] T025 [US2] Wrap CurriculumSection with AnimatedSection for scroll animation
- [ ] T026 [US2] Run CurriculumCard and CurriculumSection tests and verify they pass

**Checkpoint**: Curriculum section fully functional and testable independently

---

## Phase 5: User Story 5 - Converting to Signup (Priority: P1)

**Goal**: Footer CTA section with "Build the Future" headline, signup button, and badges

**Independent Test**: Scroll to Footer and verify CTA button links to signup page

### Tests for User Story 5

- [ ] T027 [P] [US5] Write test for FooterCTA at website/tests/components/landing/FooterCTA.test.tsx

### Implementation for User Story 5

- [ ] T028 [US5] Create FooterCTA component at website/src/components/landing/FooterCTA.tsx
- [ ] T029 [US5] Add "Build the Future" headline with large typography
- [ ] T030 [US5] Add gradient background (from-violet-900/20 rising from bottom)
- [ ] T031 [US5] Add "Create Free Profile" GlowButton linking to /signup
- [ ] T032 [US5] Add "Powered by Panaversity" and "Built with Gemini" badges
- [ ] T033 [US5] Wrap FooterCTA with AnimatedSection for scroll animation
- [ ] T034 [US5] Run FooterCTA test and verify it passes

**Checkpoint**: Footer CTA section fully functional and testable independently

---

## Phase 6: User Story 3 - Exploring Interactive Features (Priority: P2)

**Goal**: Features section with zig-zag layout showing AI Tutor chatbot demo and Personalization toggle demo

**Independent Test**: View Features section and verify zig-zag layout displays both features with animations

### Tests for User Story 3

- [ ] T035 [P] [US3] Write test for ChatbotDemo at website/tests/components/landing/ChatbotDemo.test.tsx
- [ ] T036 [P] [US3] Write test for PersonalizationDemo at website/tests/components/landing/PersonalizationDemo.test.tsx
- [ ] T037 [P] [US3] Write test for FeaturesSection at website/tests/components/landing/FeaturesSection.test.tsx

### Implementation for User Story 3

- [ ] T038 [P] [US3] Create ChatbotDemo component with animated mock chat at website/src/components/landing/ChatbotDemo.tsx
- [ ] T039 [P] [US3] Create PersonalizationDemo component with code/circuit toggle at website/src/components/landing/PersonalizationDemo.tsx
- [ ] T040 [US3] Create FeatureDemo container component with zig-zag layout at website/src/components/landing/FeatureDemo.tsx
- [ ] T041 [US3] Create FeaturesSection composing both demos at website/src/components/landing/FeaturesSection.tsx
- [ ] T042 [US3] Add "Quaternions" conversation to ChatbotDemo animation
- [ ] T043 [US3] Implement toggle state switching between Python code and circuit diagram in PersonalizationDemo
- [ ] T044 [US3] Wrap FeaturesSection with AnimatedSection for scroll animation
- [ ] T045 [US3] Run ChatbotDemo, PersonalizationDemo, and FeaturesSection tests and verify they pass

**Checkpoint**: Features section fully functional with both interactive demos

---

## Phase 7: User Story 4 - Evaluating Hardware Requirements (Priority: P2)

**Goal**: Lab section with game-style loadout showing hardware requirements and cloud alternative

**Independent Test**: View Lab section and verify all hardware items display with cloud simulation alternative mentioned

### Tests for User Story 4

- [ ] T046 [P] [US4] Write test for HardwareItem at website/tests/components/landing/HardwareItem.test.tsx
- [ ] T047 [P] [US4] Write test for LabSection at website/tests/components/landing/LabSection.test.tsx

### Implementation for User Story 4

- [ ] T048 [P] [US4] Create HardwareItem component with game-style card at website/src/components/landing/HardwareItem.tsx
- [ ] T049 [US4] Create LabSection with "Required Equipment" title at website/src/components/landing/LabSection.tsx
- [ ] T050 [US4] Add hardware data array with 3 items (Brain, Eyes, Body) in LabSection
- [ ] T051 [US4] Style LabSection as game "loadout" screen with tech aesthetic
- [ ] T052 [US4] Add "Cloud Simulation Available" note for users without hardware
- [ ] T053 [US4] Wrap LabSection with AnimatedSection for scroll animation
- [ ] T054 [US4] Run HardwareItem and LabSection tests and verify they pass

**Checkpoint**: Lab section fully functional with hardware loadout and cloud option

---

## Phase 8: User Story 6 - Smooth Scrolling Experience (Priority: P3)

**Goal**: Scroll-triggered fade-in-up animations on all sections with accessibility support

**Independent Test**: Scroll through page observing fade-in-up animations on each section, verify reduced motion preference is respected

### Implementation for User Story 6

- [ ] T055 [US6] Add useReducedMotion hook support to AnimatedSection at website/src/components/landing/AnimatedSection.tsx
- [ ] T056 [US6] Configure viewport={{ once: true }} on all AnimatedSection instances
- [ ] T057 [US6] Verify animation duration is within 500ms (spec requirement)
- [ ] T058 [US6] Test animations with prefers-reduced-motion enabled in browser

**Checkpoint**: All animations working with accessibility support

---

## Phase 9: Integration & Page Assembly

**Purpose**: Compose all sections into the final landing page

- [ ] T059 Update website/src/pages/index.tsx to import all landing components
- [ ] T060 Remove or replace website/src/pages/index.module.css (using Tailwind only)
- [ ] T061 Compose landing page with sections in order: Hero, Curriculum, Features, Lab, FooterCTA
- [ ] T062 Add deep void background (bg-slate-950) to main container
- [ ] T063 Ensure Layout wrapper preserves Docusaurus navbar functionality
- [ ] T064 Run full page visual test at all breakpoints (320px, 768px, 1024px, 1440px, 2560px)

**Checkpoint**: Complete landing page assembled and responsive

---

## Phase 10: Polish & Cross-Cutting Concerns

**Purpose**: Final quality, performance, and accessibility checks

- [ ] T065 [P] Run Lighthouse accessibility audit and achieve 90+ score
- [ ] T066 [P] Verify page renders within 3 seconds on simulated 3G
- [ ] T067 [P] Test page functionality with JavaScript disabled (content visible, links work)
- [ ] T068 Check all button glow effects render correctly
- [ ] T069 Verify glassmorphism backdrop-blur works in all target browsers
- [ ] T070 Run all component tests: npm test in website/
- [ ] T071 Run build to verify no TypeScript errors: npm run build in website/
- [ ] T072 Manual QA: click all CTAs and verify navigation works

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1: Setup ──────────────────────────────────────────────────┐
                                                                 │
Phase 2: Foundational ◄──────────────────────────────────────────┘
    │
    ├───► Phase 3: US1 Hero (P1) ─────────────┐
    │                                          │
    ├───► Phase 4: US2 Curriculum (P1) ───────┤
    │                                          │
    ├───► Phase 5: US5 Footer (P1) ───────────┤
    │                                          │
    ├───► Phase 6: US3 Features (P2) ─────────┼───► Phase 9: Integration
    │                                          │
    ├───► Phase 7: US4 Lab (P2) ──────────────┤
    │                                          │
    └───► Phase 8: US6 Animations (P3) ───────┘
                                               │
                                               ▼
                                        Phase 10: Polish
```

### User Story Dependencies

| Story | Depends On | Can Parallel With |
|-------|------------|-------------------|
| US1 Hero | Phase 2 (Foundation) | US2, US3, US4, US5 |
| US2 Curriculum | Phase 2 (Foundation) | US1, US3, US4, US5 |
| US3 Features | Phase 2 (Foundation) | US1, US2, US4, US5 |
| US4 Lab | Phase 2 (Foundation) | US1, US2, US3, US5 |
| US5 Footer | Phase 2 (Foundation) | US1, US2, US3, US4 |
| US6 Animations | Phase 2 (Foundation) | All (enhances existing) |

### Within Each User Story

1. Write tests FIRST (RED)
2. Implement components to pass tests (GREEN)
3. Refactor if needed
4. Run tests to verify

---

## Parallel Opportunities

### Phase 2 Parallelization

```bash
# All foundational components can be built in parallel:
Task: "Create AnimatedSection wrapper at website/src/components/landing/AnimatedSection.tsx"
Task: "Create GlowButton component at website/src/components/landing/GlowButton.tsx"
Task: "Write test for AnimatedSection at website/tests/components/landing/AnimatedSection.test.tsx"
Task: "Write test for GlowButton at website/tests/components/landing/GlowButton.test.tsx"
```

### User Story Parallelization (after Phase 2)

```bash
# All P1 stories can start together after foundation:
Task: "[US1] Create HeroSection"
Task: "[US2] Create CurriculumCard"
Task: "[US5] Create FooterCTA"

# All P2 stories can also parallel:
Task: "[US3] Create ChatbotDemo"
Task: "[US3] Create PersonalizationDemo"
Task: "[US4] Create HardwareItem"
```

---

## Implementation Strategy

### MVP First (P1 Stories Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational
3. Complete Phase 3: US1 Hero
4. Complete Phase 4: US2 Curriculum
5. Complete Phase 5: US5 Footer
6. **STOP and VALIDATE**: Run Phase 9 Integration with MVP sections
7. Deploy/demo if ready - landing page has core value proposition

### Full Implementation

1. Complete MVP (P1 stories)
2. Add Phase 6: US3 Features (P2)
3. Add Phase 7: US4 Lab (P2)
4. Add Phase 8: US6 Animations (P3)
5. Complete Phase 9: Full Integration
6. Complete Phase 10: Polish

---

## Summary

| Metric | Count |
|--------|-------|
| **Total Tasks** | 72 |
| **Setup Tasks** | 5 |
| **Foundational Tasks** | 5 |
| **US1 Tasks** | 8 |
| **US2 Tasks** | 8 |
| **US3 Tasks** | 11 |
| **US4 Tasks** | 9 |
| **US5 Tasks** | 8 |
| **US6 Tasks** | 4 |
| **Integration Tasks** | 6 |
| **Polish Tasks** | 8 |
| **Parallel Opportunities** | 23 tasks marked [P] |

---

## Notes

- [P] tasks = different files, no dependencies within phase
- [Story] label maps task to specific user story
- Each user story is independently completable and testable
- TDD: Write tests first, verify they fail, then implement
- Commit after each task or logical group
- MVP = Phase 1 + 2 + 3 + 4 + 5 + Integration (P1 stories only)
