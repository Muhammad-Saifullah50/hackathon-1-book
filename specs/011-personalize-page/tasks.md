# Implementation Tasks: Page Content Personalization

**Feature**: 011-personalize-page
**Branch**: `011-personalize-page`
**Date**: 2025-12-14
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

## User Story Mapping

| Story | Priority | Description | Dependencies |
|-------|----------|-------------|--------------|
| US1 | P1 | Personalize Current Page Content | Foundational |
| US2 | P2 | Visual Loading State During Personalization | US1 |
| US3 | P2 | Restore Original Content (Toggle) | US1 |
| US4 | P2 | Cross-Device Personalization Awareness | US1 |
| US5 | P3 | Personalization for Unauthenticated Users | US1 |

---

## Phase 1: Setup

**Goal**: Initialize project structure and dependencies

- [x] T001 Create database migration script in scripts/migrations/003_create_personalization_tables.sql
- [x] T002 Run database migration to create personalization_history and personalization_quota tables
- [x] T003 [P] Create backend module structure in backend/src/api/personalization/ (directory only)
- [x] T004 [P] Create agents directory structure in backend/src/ai_agents/ (directory only)
- [x] T005 [P] Create TypeScript types file in website/src/types/personalization.ts
- [x] T006 [P] Create frontend services directory in website/src/services/ (directory created)

---

## Phase 2: Foundational (Blocking)

**Goal**: Build core infrastructure required by all user stories

### Backend Models & Services

- [x] T007 Create Pydantic models in backend/src/models/personalization.py (PersonalizationRequest, PersonalizationResponse, QuotaStatus, etc.)
- [x] T008 Create personalization service in backend/src/services/personalization_service.py with quota management functions
- [x] T009 [P] Create hash utility functions in backend/src/services/personalization_service.py (compute_profile_hash, compute_content_hash)
- [x] T010 Create personalization agent with dynamic instructions in backend/src/ai_agents/personalization_agent.py
- [x] T011 [P] Create markdown preservation utilities in backend/src/ai_agents/personalization_agent.py (extract_preservable_blocks, restore_preserved_blocks)

### Backend API Routes

- [x] T012 Create API router file in backend/src/api/personalization/routes.py with route stubs
- [x] T013 [P] Create request validation dependencies (integrated into routes.py)
- [x] T014 Register personalization router in backend/main.py

### Frontend Services

- [x] T015 Create hash utility functions in website/src/utils/hashUtils.ts (computeProfileHash, computeContentHash)
- [x] T016 Create personalization API service in website/src/services/personalizationService.ts (API client methods)
- [x] T017 Create localStorage cache manager in website/src/services/personalizationService.ts (LRU eviction, cache operations)

---

## Phase 3: User Story 1 - Personalize Current Page Content (P1 - MVP)

**Goal**: Enable users to click "Personalize Page" and receive tailored content

**Independent Test**: Click personalize button on any doc page, verify content is personalized based on profile (beginner gets simplified, advanced gets technical)

### Backend Implementation

- [x] T018 [US1] Implement POST /personalization/personalize endpoint in backend/src/api/personalization/routes.py
- [x] T019 [US1] Implement personalize_content() method in backend/src/ai_agents/personalization_agent.py
- [x] T020 [US1] Implement quota check and decrement logic in backend/src/services/personalization_service.py
- [x] T021 [US1] Implement history record creation (upsert) in backend/src/services/personalization_service.py
- [x] T022 [US1] Implement GET /personalization/quota endpoint in backend/src/api/personalization/routes.py

### Frontend Implementation

- [x] T023 [US1] Create usePersonalization hook in website/src/hooks/usePersonalization.ts with personalize() function
- [x] T024 [US1] Update PersonalizationBar component in website/src/components/profile/PersonalizationBar.tsx to call personalize API
- [x] T025 [US1] Implement page content extraction from DOM in PersonalizationBar.tsx
- [x] T026 [US1] Implement personalized content display (replace page content) in PersonalizationBar.tsx
- [x] T027 [US1] Display quota status in PersonalizationBar component

### Tests (TDD per Constitution)

- [x] T028 [P] [US1] Write unit tests for personalization_service in backend/tests/unit/services/test_personalization_service.py
- [x] T029 [P] [US1] Write integration tests for personalization API in backend/tests/integration/api/test_personalization_api.py
- [x] T030 [P] [US1] Write unit tests for usePersonalization hook in website/tests/unit/hooks/usePersonalization.test.ts

---

## Phase 4: User Story 2 - Visual Loading State (P2)

**Goal**: Show loading indicator during personalization

**Independent Test**: Click personalize, verify loading spinner appears within 200ms, dismisses when content arrives

### Implementation

- [x] T031 [US2] Add loading state to usePersonalization hook in website/src/hooks/usePersonalization.ts
- [x] T032 [US2] Implement loading indicator UI (shimmer/spinner) in website/src/components/profile/PersonalizationBar.tsx
- [x] T033 [US2] Add error state handling and retry button in PersonalizationBar.tsx
- [x] T034 [US2] Implement timeout handling (>15 seconds) with user feedback in usePersonalization.ts

### Tests

- [x] T035 [P] [US2] Write tests for loading states in website/tests/unit/components/profile/PersonalizationBar.test.tsx

---

## Phase 5: User Story 3 - Restore Original Content (P2)

**Goal**: Allow users to toggle between original and personalized content

**Independent Test**: Personalize content, click "Show Original", verify original reappears instantly from cache

### Backend Implementation

- [x] T036 [US3] Ensure original content is stored in localStorage cache (in personalizationService.ts)

### Frontend Implementation

- [x] T037 [US3] Add toggle state (original/personalized) to usePersonalization hook in website/src/hooks/usePersonalization.ts
- [x] T038 [US3] Implement "Show Original" / "Show Personalized" toggle button in PersonalizationBar.tsx
- [x] T039 [US3] Implement instant toggle from localStorage cache in personalizationService.ts
- [x] T040 [US3] Detect previously personalized pages on page load and show "View Personalized" option

### Tests

- [x] T041 [P] [US3] Write tests for toggle functionality in website/tests/unit/hooks/usePersonalization.test.ts

---

## Phase 6: User Story 4 - Cross-Device Personalization Awareness (P2)

**Goal**: Enable cross-device awareness with free re-personalization

**Independent Test**: Personalize on Device A, login on Device B, verify "Re-personalize (Free)" appears

### Backend Implementation

- [x] T042 [US4] Implement GET /personalization/history endpoint in backend/src/api/personalization/routes.py
- [x] T043 [US4] Implement GET /personalization/history/{pageUrl} endpoint in backend/src/api/personalization/routes.py
- [x] T044 [US4] Implement is_free_repersonalization logic (skip quota decrement) in personalization_service.py
- [x] T045 [US4] Implement profile staleness detection (profile_hash comparison) in personalization_service.py

### Frontend Implementation

- [x] T046 [US4] Add fetchHistory() method to personalizationService.ts
- [x] T047 [US4] Implement cross-device detection on page load in usePersonalization.ts
- [x] T048 [US4] Add "Re-personalize (Free)" button variant in PersonalizationBar.tsx
- [x] T049 [US4] Add "Re-personalize with new profile (Free)" for stale profile detection

### Tests

- [x] T050 [P] [US4] Write tests for history API in backend/tests/integration/api/test_personalization_api.py
- [x] T051 [P] [US4] Write tests for cross-device detection in website/tests/unit/hooks/usePersonalization.test.ts

---

## Phase 7: User Story 5 - Unauthenticated Users (P3)

**Goal**: Guide unauthenticated users toward login/signup

**Independent Test**: View page as logged-out user, verify "Log in to personalize" message appears

### Frontend Implementation

- [x] T052 [US5] Add authentication check to PersonalizationBar rendering logic
- [x] T053 [US5] Implement "Log in to personalize" message with login link in PersonalizationBar.tsx
- [x] T054 [US5] Implement "Complete your profile" prompt for users without profile in PersonalizationBar.tsx

### Tests

- [x] T055 [P] [US5] Write tests for unauthenticated state rendering in PersonalizationBar.test.tsx

---

## Phase 8: Polish & Cross-Cutting Concerns

**Goal**: Handle edge cases, improve UX, finalize integration

### Edge Cases

- [x] T056 Implement request cancellation on navigation in usePersonalization.ts
- [x] T057 Implement localStorage LRU eviction when limit reached in personalizationService.ts
- [x] T058 Handle empty/short content edge case (<100 chars) in personalization_service.py
- [x] T059 Handle server history API unavailable (graceful degradation) in usePersonalization.ts
- [x] T060 Detect original content changes (hash mismatch) and offer re-personalization

### Profile Change Handling

- [x] T061 Implement cache invalidation on profile update in usePersonalization.ts
- [x] T062 Listen to profile update events and clear localStorage cache

### E2E Tests

- [x] T063 Write E2E test for full personalization flow in website/tests/e2e/personalization.spec.ts
- [x] T064 Write E2E test for cross-device scenario (simulated) in personalization.spec.ts

### Documentation

- [x] T065 Update quickstart.md with any implementation changes discovered during development

---

## Dependency Graph

```
Phase 1 (Setup)
    │
    ▼
Phase 2 (Foundational) ─────────────────────────────────┐
    │                                                    │
    ▼                                                    │
Phase 3 (US1: Core Personalization) ◄── MVP Milestone   │
    │                                                    │
    ├─────────────┬─────────────┬─────────────┐         │
    ▼             ▼             ▼             ▼         │
Phase 4       Phase 5       Phase 6       Phase 7       │
(US2:Loading) (US3:Toggle)  (US4:Cross)   (US5:Unauth)  │
    │             │             │             │         │
    └─────────────┴─────────────┴─────────────┘         │
                        │                               │
                        ▼                               │
                  Phase 8 (Polish) ◄────────────────────┘
```

## Parallel Execution Opportunities

### Within Phase 2 (Foundational)
```
T007 ──┐
T009 ──┼──► T008 (service depends on models)
T010 ──┤
T011 ──┘

T015 ──┐
       ├──► T016, T017 can run in parallel
T005 ──┘
```

### Within Phase 3 (US1)
```
Backend:                    Frontend:
T018 ──► T019 ──► T020     T023 ──► T024 ──► T025
         │                          │
         ▼                          ▼
        T021, T022                 T026, T027

Tests (parallel with implementation):
T028 ─┬─ T029 ─┬─ T030
```

### Phases 4-7 (US2-US5) can run in parallel after US1 completion
```
Phase 4 (US2) ─┬─ Phase 5 (US3) ─┬─ Phase 6 (US4) ─┬─ Phase 7 (US5)
               │                 │                 │
               └─────────────────┴─────────────────┘
                              │
                              ▼
                         Phase 8
```

---

## Implementation Strategy

### MVP Scope (Recommended First Delivery)
- **Phase 1 + Phase 2 + Phase 3 (US1)** = Core personalization working
- Delivers: Button click → API call → Personalized content displayed
- Excludes: Loading states, toggle, cross-device, unauthenticated handling

### Incremental Delivery Order
1. **MVP**: US1 (Core personalization)
2. **UX Polish**: US2 (Loading) + US3 (Toggle)
3. **Cross-Device**: US4 (History API + free re-personalization)
4. **Growth**: US5 (Unauthenticated user guidance)
5. **Hardening**: Phase 8 (Edge cases, E2E tests)

---

## Task Summary

| Phase | Story | Task Count | Parallelizable |
|-------|-------|------------|----------------|
| 1 | Setup | 6 | 4 |
| 2 | Foundational | 11 | 5 |
| 3 | US1 (P1) | 13 | 5 |
| 4 | US2 (P2) | 5 | 1 |
| 5 | US3 (P2) | 6 | 1 |
| 6 | US4 (P2) | 10 | 2 |
| 7 | US5 (P3) | 4 | 1 |
| 8 | Polish | 10 | 0 |
| **Total** | | **65** | **19** |

---

## Acceptance Criteria per Story

| Story | Independent Test | Key Verification |
|-------|------------------|------------------|
| US1 | Click personalize → content changes | Beginner profile → simplified; Advanced → technical |
| US2 | Click personalize → spinner appears | Loading within 200ms, dismisses on completion |
| US3 | Personalize → Show Original → instant | Toggle < 100ms (cached) |
| US4 | Device A personalize → Device B shows "Re-personalize (Free)" | Quota not decremented |
| US5 | Logged out → "Log in to personalize" | Link to login page visible |
