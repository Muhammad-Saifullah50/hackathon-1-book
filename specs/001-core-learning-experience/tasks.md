---
description: "Task list for Core Learning Experience implementation"
---

# Tasks: Core Learning Experience

**Input**: Design documents from `/specs/001-core-learning-experience/`
**Prerequisites**: plan.md (required), spec.md (required)

**Tests**: **MANDATORY TDD**. All user story phases must start with writing failing tests as per Constitution Section 6.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Initialize Docusaurus v3 project with TypeScript in `website/`
- [X] T002 Install and configure Tailwind CSS via PostCSS in `website/tailwind.config.js` and `website/docusaurus.config.ts`
- [X] T003 Initialize Shadcn UI structure (components.json, lib/utils.ts) in `website/src/`
- [X] T004 [P] Configure Jest and React Testing Library in `tests/unit/`
- [X] T005 [P] Configure Playwright for E2E testing in `tests/e2e/`
- [X] T006 Setup global layout wrapper in `website/src/theme/Layout/index.tsx` for Shadcn integration

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T007 Define Global React Context for Personalization/Language in `website/src/context/LearningContext.tsx`
- [ ] T008 [P] Implement base Shadcn Button primitive in `website/src/components/ui/button.tsx`
- [ ] T009 [P] Implement base Shadcn Card primitive in `website/src/components/ui/card.tsx`
- [ ] T010 [P] Implement base Shadcn ScrollArea primitive in `website/src/components/ui/scroll-area.tsx`
- [ ] T011 Verify TDD infrastructure by running a simple passing test in `tests/unit/example.test.tsx`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Curriculum Sidebar Navigation (Priority: P1)

**Goal**: Persistent, organized sidebar mirroring the 13-week curriculum

**Independent Test**: Verify sidebar structure mirrors curriculum JSON and highlights active lesson

### Tests for User Story 1 (MANDATORY TDD)

- [X] T012 [P] [US1] Write unit test for Sidebar component curriculum rendering in `tests/unit/components/Sidebar.test.tsx`

### Implementation for User Story 1

- [X] T013 [US1] Define curriculum structure JSON in `website/src/data/curriculum.json`
- [X] T014 [US1] Create custom Sidebar component (swizzled or custom) in `website/src/components/Sidebar/index.tsx`
- [X] T015 [US1] Integrate curriculum JSON into Sidebar component logic
- [X] T016 [US1] Style sidebar with Tailwind for "clean, modern" look (generous whitespace)

**Checkpoint**: Sidebar functional and tested

---

## Phase 4: User Story 11 - Structured Curriculum Delivery (Priority: P1)

**Goal**: Structure content into 4 key modules (Foundations, Digital Twin, AI-Robot Brain, Embodied Intelligence)

**Independent Test**: Verify Module structure exists and contains correct lesson placeholders

### Tests for User Story 11 (MANDATORY TDD)

- [X] T017 [P] [US11] Write E2E test verifying Module 1-4 navigation in `tests/e2e/curriculum.spec.ts`

### Implementation for User Story 11

- [X] T018 [P] [US11] Create directory structure for Module 1 (Foundations) in `website/docs/module-1/`
- [X] T019 [P] [US11] Create directory structure for Module 2 (Digital Twin) in `website/docs/module-2/`
- [X] T020 [P] [US11] Create directory structure for Module 3 (AI-Robot Brain) in `website/docs/module-3/`
- [X] T021 [P] [US11] Create directory structure for Module 4 (Embodied Intelligence) in `website/docs/module-4/`
- [X] T022 [US11] Create placeholder Markdown files for Lesson 1.1, 1.2, etc.

**Checkpoint**: Content structure established

---

## Phase 5: User Story 2 - Clean & Modern Typography (Priority: P1)

**Goal**: High-readability typography with generous whitespace

**Independent Test**: Visual inspection against design standards

### Tests for User Story 2 (MANDATORY TDD)

- [X] T023 [P] [US2] Write unit test checks for typography class application in `tests/unit/theme/Typography.test.tsx`

### Implementation for User Story 2

- [X] T024 [US2] Configure Tailwind typography plugin in `website/tailwind.config.js`
- [X] T025 [US2] Implement global typography styles (h1-h6, p, blockquote) in `website/src/css/custom.css`
- [X] T026 [US2] Verify generous whitespace (margins/padding) in standard layout

**Checkpoint**: Typography applied globally

---

## Phase 6: User Story 3 - Interactive Code Blocks (Priority: P1)

**Goal**: Syntax highlighted code blocks with Copy button

**Independent Test**: Copy button places code in clipboard

### Tests for User Story 3 (MANDATORY TDD)

- [X] T027 [P] [US3] Write unit test for CodeBlock copy functionality in `tests/unit/components/CodeBlock.test.tsx`

### Implementation for User Story 3

- [X] T028 [US3] Swizzle/Create CodeBlock component in `website/src/theme/CodeBlock/index.tsx`
- [X] T029 [US3] Implement syntax highlighting logic (using Prism or Docusaurus default)
- [X] T030 [US3] Implement Copy to Clipboard button logic in CodeBlock component

**Checkpoint**: Code blocks interactive

---

## Phase 7: User Story 4 - Linear Navigation (Priority: P1)

**Goal**: Next/Previous buttons at bottom of pages

**Independent Test**: Verify navigation flow between sequential lessons

### Tests for User Story 4 (MANDATORY TDD)

- [X] T031 [P] [US4] Write unit test for Paginator component logic in `tests/unit/components/Paginator.test.tsx`

### Implementation for User Story 4

- [X] T032 [US4] Swizzle/Customize DocPaginator component in `website/src/theme/DocPaginator/index.tsx`
- [X] T033 [US4] Ensure Next/Prev logic respects curriculum order defined in `website/src/data/curriculum.json`

**Checkpoint**: Linear navigation functional

---

## Phase 8: User Story 5 - Responsive Interface (Priority: P1)

**Goal**: Functional layout on Mobile/Tablet

**Independent Test**: Sidebar collapses/overlays on small screens

### Tests for User Story 5 (MANDATORY TDD)

- [X] T034 [P] [US5] Write E2E test for mobile viewport layout in `tests/e2e/responsive.spec.ts`

### Implementation for User Story 5

- [X] T035 [US5] Implement mobile sidebar toggle in `website/src/components/Sidebar/MobileSidebar.tsx` (using Shadcn Sheet/Dialog)
- [X] T036 [US5] Apply responsive Tailwind classes (md:, lg:) to main layout containers

**Checkpoint**: Mobile responsive layout functional

---

## Phase 9: User Story 6 - Global Search (Priority: P2)

**Goal**: Instant definition lookup

**Independent Test**: Search for "URDF" returns correct result

### Tests for User Story 6 (MANDATORY TDD)

- [X] T037 [P] [US6] Write E2E test for search functionality in `tests/e2e/search.spec.ts`

### Implementation for User Story 6

- [X] T038 [US6] Configure Docusaurus local search plugin or Algolia in `website/docusaurus.config.ts`
- [X] T039 [US6] Customize SearchBar component styles in `website/src/theme/SearchBar/index.tsx` if needed for visual consistency

**Checkpoint**: Search functional

---

## Phase 10: User Story 7 - Visual Lab/Capstone Distinctions (Priority: P2)

**Goal**: Visually distinct styles for Lab/Capstone pages

**Independent Test**: "Lab" pages have distinct styling

### Tests for User Story 7 (MANDATORY TDD)

- [X] T040 [P] [US7] Write unit test for Admonition/Callout rendering in `tests/unit/components/Admonition.test.tsx`

### Implementation for User Story 7

- [X] T041 [US7] Create custom Admonition types for "Lab" and "Capstone" in `website/docusaurus.config.ts`
- [X] T042 [US7] Style Lab/Capstone banners with distinct colors/icons in `website/src/css/custom.css`

**Checkpoint**: Lab/Capstone visually distinct

---

## Phase 11: User Story 8 - Robot Imagery (Priority: P2)

**Goal**: Embedded high-quality robot images

**Independent Test**: Images load correctly next to text

### Tests for User Story 8 (MANDATORY TDD)

- [X] T043 [P] [US8] Write unit test for Image component responsiveness in `tests/unit/components/Image.test.tsx`

### Implementation for User Story 8

- [X] T044 [US8] Add Unitree Go2/G1 placeholder images to `website/static/img/robots/`
- [X] T045 [US8] Implement optimized Image component in `website/src/components/ui/image.tsx`

**Checkpoint**: Robot images embeddable

---

## Phase 12: User Story 9 - Content Personalization (Priority: P3)

**Goal**: Toggle between Software/Hardware explanation modes

**Independent Test**: Toggling mode changes text content

### Tests for User Story 9 (MANDATORY TDD)

- [ ] T046 [P] [US9] Write unit test for Personalization Toggle in `tests/unit/components/Personalization.test.tsx`

### Implementation for User Story 9

- [ ] T047 [US9] Create PersonalizationToggle component in `website/src/components/Features/PersonalizationToggle.tsx`
- [ ] T048 [US9] Create AdaptiveText component that consumes LearningContext in `website/src/components/Features/AdaptiveText.tsx`
- [ ] T049 [US9] Add "Personalize This Chapter" button to top of Doc layout

**Checkpoint**: Personalization toggle functional

---

## Phase 13: User Story 10 - Urdu Translation (Priority: P3)

**Goal**: Toggle text language to Urdu

**Independent Test**: Toggling language updates text content

### Tests for User Story 10 (MANDATORY TDD)

- [ ] T050 [P] [US10] Write unit test for Language Toggle in `tests/unit/components/LanguageToggle.test.tsx`

### Implementation for User Story 10

- [ ] T051 [US10] Create LanguageToggle component in `website/src/components/Features/LanguageToggle.tsx`
- [ ] T052 [US10] Configure Docusaurus i18n for Urdu (ur) in `website/docusaurus.config.ts`
- [ ] T053 [US10] Integrate LanguageToggle into Navbar

**Checkpoint**: Urdu translation toggle functional

---

## Phase 14: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements and cleanup

- [X] T054 Run full E2E regression suite with Playwright
- [ ] T055 [P] Audit accessibility (color contrast, aria-labels)
- [ ] T056 [P] Optimize image assets in `website/static/img/`
- [ ] T057 Final code cleanup and linting check

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies
- **Foundational (Phase 2)**: Depends on Phase 1 - BLOCKS all stories
- **User Stories (Phase 3-13)**: Can be largely parallelized after Phase 2, though P1 stories should be prioritized.
- **Polish (Phase 14)**: Depends on all user stories

### Implementation Strategy

#### MVP First (P1 Stories)
1. Complete Setup + Foundational
2. Implement Sidebar (US1) + Content Structure (US11) + Typography (US2) + Nav (US4) + Mobile (US5) + Code Blocks (US3)
3. **Validate**: Fully functional course reader with basic content structure.

#### Enhanced Feature Set (P2 Stories)
1. Add Search (US6) + Lab Visuals (US7) + Robot Images (US8)
2. **Validate**: Richer, more navigable learning experience.

#### Advanced Features (P3 Stories)
1. Add Personalization (US9) + Urdu Translation (US10)
2. **Validate**: Fully adaptive and accessible platform.