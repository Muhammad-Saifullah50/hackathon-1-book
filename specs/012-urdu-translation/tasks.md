# Tasks: Urdu Translation Bar

**Input**: Design documents from `/specs/012-urdu-translation/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/api-spec.yaml

**Tests**: Included per constitution TDD mandate (Red-Green-Refactor cycle)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/src/`, `backend/tests/`
- **Frontend**: `website/src/`, `website/tests/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create database migration file in backend/scripts/migrations/004_create_translation_tables.sql
- [x] T002 [P] Create translation TypeScript types in website/src/types/translation.ts
- [x] T003 [P] Create backend translation models in backend/src/models/translation.py
- [x] T004 [P] Create translation API router module in backend/src/api/translation/ (skipped __init__.py per user preference)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T005 Run database migration to create translation_history and translation_quota tables
- [x] T006 Create urdu_translation_agent with Gemini model in backend/src/ai_agents/translation_agent.py
- [x] T007 [P] Create IP extraction helper in backend/src/api/translation/dependencies.py
- [x] T008 [P] Create translation service skeleton in backend/src/services/translation_service.py
- [x] T009 [P] Create translation frontend service skeleton in website/src/services/translationService.ts
- [x] T010 Register translation router in backend/main.py

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Translate Page Content to Urdu (Priority: P1) 🎯 MVP

**Goal**: Users can click "Translate to Urdu" button and receive translated content with code blocks preserved

**Independent Test**: Click translate button on any doc page, verify Urdu content appears with RTL direction, code blocks unchanged

### Tests for User Story 1 (TDD - Write First, Must Fail)

- [x] T011 [P] [US1] Write unit test for translation_service.translate() in backend/tests/unit/services/test_translation_service.py
- [x] T012 [P] [US1] Write integration test for POST /api/translate/urdu in backend/tests/integration/api/test_translation_api.py
- [x] T013 [P] [US1] Write unit test for useTranslation hook translate function in website/tests/unit/hooks/useTranslation.test.ts

### Implementation for User Story 1

- [x] T014 [US1] Implement translate_content() in backend/src/services/translation_service.py
- [x] T015 [US1] Implement POST /api/translate/urdu endpoint in backend/src/api/translation/routes.py
- [x] T016 [P] [US1] Create useTranslation hook with translate() function in website/src/hooks/useTranslation.ts
- [x] T017 [P] [US1] Create translationService.translate() API client in website/src/services/translationService.ts
- [x] T018 [US1] Create TranslationBar component with translate button in website/src/components/translation/TranslationBar.tsx
- [x] T019 [US1] Add RTL CSS styling for translated content container in website/src/components/translation/TranslationBar.tsx
- [x] T020 [US1] Modify Heading wrapper to render TranslationBar below PersonalizationBar in website/src/theme/Heading/index.tsx

**Checkpoint**: User Story 1 complete - users can translate pages to Urdu

---

## Phase 4: User Story 2 - Visual Loading State During Translation (Priority: P2)

**Goal**: Users see clear loading indicator during translation with error handling

**Independent Test**: Click translate, verify loading spinner appears immediately, disappears when done or shows error on failure

### Tests for User Story 2 (TDD - Write First, Must Fail)

- [x] T021 [P] [US2] Write unit test for loading/error states in useTranslation hook in website/tests/unit/hooks/useTranslation.test.ts
- [x] T022 [P] [US2] Write component test for TranslationBar loading states in website/tests/unit/components/translation/TranslationBar.test.tsx

### Implementation for User Story 2

- [x] T023 [US2] Add loading state rendering to TranslationBar component in website/src/components/translation/TranslationBar.tsx
- [x] T024 [US2] Add error state rendering with retry button to TranslationBar in website/src/components/translation/TranslationBar.tsx
- [x] T025 [US2] Implement error handling in useTranslation hook in website/src/hooks/useTranslation.ts
- [x] T026 [US2] Add timeout handling (30s) to translationService in website/src/services/translationService.ts

**Checkpoint**: User Story 2 complete - users see loading/error states

---

## Phase 5: User Story 3 - Toggle Between Original and Translated Content (Priority: P2)

**Goal**: Users can toggle between original and translated views with instant switching from cache

**Independent Test**: Translate page, click "Show Original", verify original appears instantly, click "Show Translated", verify cached translation appears

### Tests for User Story 3 (TDD - Write First, Must Fail)

- [x] T027 [P] [US3] Write unit test for toggleView() and cache functions in website/tests/unit/hooks/useTranslation.test.ts
- [x] T028 [P] [US3] Write unit test for localStorage cache operations in website/tests/unit/services/translationService.test.ts
- [x] T029 [P] [US3] Write component test for toggle button states in website/tests/unit/components/translation/TranslationBar.test.tsx

### Implementation for User Story 3

- [x] T030 [US3] Implement localStorage caching with 30-day expiry in website/src/services/translationService.ts
- [x] T031 [US3] Implement LRU eviction for localStorage cache in website/src/services/translationService.ts
- [x] T032 [US3] Implement toggleView() in useTranslation hook in website/src/hooks/useTranslation.ts
- [x] T033 [US3] Implement hasCachedVersion detection on page load in website/src/hooks/useTranslation.ts
- [x] T034 [US3] Add toggle button UI (Show Original/Show Translated) to TranslationBar in website/src/components/translation/TranslationBar.tsx
- [x] T035 [US3] Implement dual-container content rendering pattern in website/src/components/translation/TranslationBar.tsx

**Checkpoint**: User Story 3 complete - users can toggle between original and translated

---

## Phase 6: User Story 4 - Translation for All Users with Quota (Priority: P2)

**Goal**: Both authenticated and anonymous users can translate with 5/day quota enforced by user_id or IP

**Independent Test**: Translate without logging in (verify works), translate 5 times (verify quota message on 6th attempt)

### Tests for User Story 4 (TDD - Write First, Must Fail)

- [x] T036 [P] [US4] Write unit test for quota checking/updating in backend/tests/unit/services/test_translation_service.py
- [x] T037 [P] [US4] Write integration test for quota enforcement in backend/tests/integration/api/test_translation_api.py
- [x] T038 [P] [US4] Write integration test for anonymous vs authenticated user handling in backend/tests/integration/api/test_translation_api.py
- [x] T039 [P] [US4] Write unit test for quota display in TranslationBar in website/tests/unit/components/translation/TranslationBar.test.tsx

### Implementation for User Story 4

- [x] T040 [US4] Implement get_quota_status() in backend/src/services/translation_service.py
- [x] T041 [US4] Implement check_and_update_quota() in backend/src/services/translation_service.py
- [x] T042 [US4] Implement GET /api/translate/quota endpoint in backend/src/api/translation/routes.py
- [x] T043 [US4] Implement IP-based quota for anonymous users in backend/src/services/translation_service.py
- [x] T044 [US4] Add quota fetching to useTranslation hook in website/src/hooks/useTranslation.ts
- [x] T045 [US4] Add quota display (X/5 remaining) to TranslationBar in website/src/components/translation/TranslationBar.tsx
- [x] T046 [US4] Add quota exceeded state UI to TranslationBar in website/src/components/translation/TranslationBar.tsx
- [x] T047 [US4] Implement GET /api/translate/history endpoint for cross-device awareness in backend/src/api/translation/routes.py
- [x] T048 [US4] Implement save_translation_history() in backend/src/services/translation_service.py

**Checkpoint**: User Story 4 complete - all users can translate with quota enforcement

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: E2E tests, edge cases, and refinements

- [ ] T049 [P] Write E2E test for full translation flow in website/tests/e2e/translation.spec.ts
- [ ] T050 [P] Write E2E test for quota exceeded scenario in website/tests/e2e/translation.spec.ts
- [ ] T051 [P] Write E2E test for toggle view scenario in website/tests/e2e/translation.spec.ts
- [ ] T052 Handle edge case: content too short (<50 chars) in backend/src/services/translation_service.py
- [ ] T053 Handle edge case: stale cache detection (content hash mismatch) in website/src/services/translationService.ts
- [ ] T054 Add request cancellation on navigation in website/src/hooks/useTranslation.ts
- [ ] T055 Verify all tests pass and run quickstart.md validation

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - US1 (P1): Must complete first (core functionality)
  - US2, US3, US4 (P2): Can proceed in parallel after US1 or sequentially
- **Polish (Phase 7)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational - No dependencies on other stories
- **User Story 2 (P2)**: Can start after US1 - Enhances loading/error states
- **User Story 3 (P2)**: Can start after US1 - Adds caching/toggle
- **User Story 4 (P2)**: Can start after US1 - Adds quota management

### Within Each User Story

- Tests MUST be written and FAIL before implementation (TDD)
- Backend before frontend within each story
- Models/services before API endpoints
- Hooks before components (frontend)
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel
- Tests for each user story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members (after US1)

---

## Parallel Example: User Story 1 Tests

```bash
# Launch all tests for User Story 1 together (TDD - must fail first):
Task: "Write unit test for translation_service.translate()"
Task: "Write integration test for POST /api/translate/urdu"
Task: "Write unit test for useTranslation hook translate function"
```

## Parallel Example: User Story 1 Implementation

```bash
# After tests written, launch parallel implementation:
Task: "Create useTranslation hook with translate() function"
Task: "Create translationService.translate() API client"
# Then sequentially:
Task: "Implement translate_content() in backend service"
Task: "Implement POST /api/translate/urdu endpoint"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL)
3. Complete Phase 3: User Story 1 (core translation)
4. **STOP and VALIDATE**: Test translation works independently
5. Deploy/demo if ready - users can translate pages!

### Incremental Delivery

1. Setup + Foundational → Foundation ready
2. Add User Story 1 → Test → Deploy (MVP - basic translation works!)
3. Add User Story 2 → Test → Deploy (loading/error UX)
4. Add User Story 3 → Test → Deploy (caching/toggle)
5. Add User Story 4 → Test → Deploy (quota management)
6. Add Polish → Full feature complete

### Task Count Summary

| Phase | Task Count |
|-------|------------|
| Setup | 4 |
| Foundational | 6 |
| User Story 1 (P1) | 10 |
| User Story 2 (P2) | 6 |
| User Story 3 (P2) | 9 |
| User Story 4 (P2) | 13 |
| Polish | 7 |
| **Total** | **55** |

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing (TDD mandate)
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
