# Tasks: Authentication & Detailed Profiling

**Input**: Design documents from `/specs/008-learner-auth-profile/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The feature specification explicitly implies a TDD approach with acceptance criteria, so tests are included.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `website/src/`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize development environment for backend and frontend, and prepare Supabase.

- [X] T001 Initialize Supabase local environment as per `specs/008-learner-auth-profile/quickstart.md`.
- [X] T002 Create Supabase `profiles` table schema and migration in `supabase/migrations/` as per `specs/008-learner-auth-profile/data-model.md`.
- [X] T003 Configure backend `.env` variables in `backend/.env` as per `specs/008-learner-auth-profile/quickstart.md`.
- [X] T004 Configure frontend `.env` variables in `website/.env` as per `specs/008-learner-auth-profile/quickstart.md`.
- [X] T005 Install Python dependencies in `backend/requirements.txt`.
- [X] T006 Install Node.js dependencies in `website/package.json`.

---

## Phase 2: Foundational (Authentication Core)

**Purpose**: Implement core authentication services and basic UI elements, enabling user login/signup.

- [X] T007 [P] Create `backend/src/api/auth/` directory.
- [X] T008 [P] Create `backend/src/api/auth/routes.py` for authentication endpoints.
- [X] T009 [P] Create `backend/src/services/auth_service.py` file.
- [X] T010 [P] Create unit tests for `auth_service.py` in `backend/tests/unit/services/test_auth_service.py`.
- [X] T011 Implement `auth_service.py` for user registration, login, logout.
- [X] T012 Implement `POST /auth/signup` endpoint in `backend/src/api/auth/routes.py` (referencing `specs/008-learner-auth-profile/contracts/auth_signup.yaml`).
- [X] T013 Implement `POST /auth/login` endpoint in `backend/src/api/auth/routes.py` (referencing `specs/008-learner-auth-profile/contracts/auth_login.yaml`).
- [X] T014 Implement `POST /auth/logout` endpoint in `backend/src/api/auth/routes.py` (referencing `specs/008-learner-auth-profile/contracts/auth_logout.yaml`).
- [X] T015 Implement `GET /auth/me` endpoint in `backend/src/api/auth/routes.py` (referencing `specs/008-learner-auth-profile/contracts/auth_me.yaml`).
- [X] T016 [P] Create integration tests for authentication API in `backend/tests/integration/api/test_auth_api.py`.
- [X] T017 [P] Create `website/src/hooks/useAuth.ts` file.
- [X] T018 [P] Create unit tests for `useAuth.ts` in `website/tests/unit/hooks/test_useAuth.test.ts`.
- [X] T019 Implement `useAuth.ts` hook for managing authentication state.
- [X] T020 [P] Create `website/src/components/auth/` directory.
- [X] T021 [P] Create `website/src/components/auth/LoginForm.tsx` component.
- [X] T022 [P] Create `website/src/components/auth/SignupForm.tsx` component.
- [X] T023 [P] Create unit tests for `LoginForm.tsx` in `website/tests/unit/components/auth/LoginForm.test.ts`.
- [X] T024 [P] Create unit tests for `SignupForm.tsx` in `website/tests/unit/components/auth/SignupForm.test.ts`.

---

## Phase 3: User Story 1 - The "Learner Profile" Wizard (Priority: P1) 🎯 MVP

**Goal**: Enable users to create a detailed learner profile through a multi-step wizard.

**Independent Test**: A new user can complete the wizard (or skip it) and have their profile data accurately stored and retrieved.

### Implementation for User Story 1

- [X] T025 [US1] Create `backend/src/models/profile.py` (Pydantic model for User Profile).
- [X] T026 [P] [US1] Create unit tests for `profile.py` model validation in `backend/tests/unit/models/test_profile.py`.
- [X] T027 [US1] Create `backend/src/api/profile/` directory.
- [X] T028 [US1] Create `backend/src/api/profile/routes.py` for profile API endpoints.
- [X] T029 [US1] Create `backend/src/services/profile_service.py` file.
- [X] T030 [P] [US1] Create unit tests for `profile_service.py` in `backend/tests/unit/services/test_profile_service.py`.
- [X] T031 [US1] Implement `profile_service.py` for profile CRUD operations (create, get, update).
- [X] T032 Implement `POST /profile` endpoint in `backend/src/api/profile/routes.py` (referencing `specs/008-learner-auth-profile/contracts/profile_crud.yaml`).
- [X] T033 Implement `GET /profile` endpoint in `backend/src/api/profile/routes.py` (referencing `specs/008-learner-auth-profile/contracts/profile_crud.yaml`).
- [X] T034 [P] [US1] Create integration tests for profile API in `backend/tests/integration/api/test_profile_api.py`.
- [X] T035 [US1] Create `website/src/data/profile-schema.ts` (Zod schema for profile validation).
- [X] T036 [P] [US1] Create unit tests for `profile-schema.ts` in `website/tests/unit/data/test_profile-schema.test.ts`.
- [X] T037 [US1] Create `website/src/hooks/useProfile.ts` file.
- [X] T038 [P] [US1] Create unit tests for `useProfile.ts` in `website/tests/unit/hooks/test_useProfile.test.ts`.
- [X] T039 [US1] Implement `useProfile.ts` hook for profile data management.
- [X] T040 [US1] Create `website/src/pages/signup-wizard/` directory.
- [X] T041 [US1] Create `website/src/pages/signup-wizard/index.tsx` page.
- [X] T042 [P] [US1] Create `website/src/components/profile/` directory.
- [X] T043 [P] [US1] Create `website/src/components/profile/ProfileWizardStep1.tsx` (Basics).
- [X] T044 [P] [US1] Create `website/src/components/profile/ProfileWizardStep2.tsx` (Background).
- [X] T045 [P] [US1] Create `website/src/components/profile/ProfileWizardStep3.tsx` (Strategy).
- [X] T046 [P] [US1] Create unit tests for wizard steps in `website/tests/unit/components/profile/`.
- [X] T047 [US1] Implement multi-step wizard logic in `website/src/pages/signup-wizard/index.tsx`.
- [X] T048 [US1] Implement "Skip for now" functionality in `website/src/pages/signup-wizard/index.tsx`.
- [X] T049 [US1] [P] Create E2E tests for the "Learner Profile" wizard flow in `tests/e2e/profile_wizard.spec.ts`.

---

## Phase 4: User Story 2 - The "Personalize" Trigger (Priority: P2)

**Goal**: Display a persistent personalization bar and trigger personalization actions.

**Independent Test**: The personalization bar is visible in Chapter 1 and clicking it triggers the expected (deferred) action.

### Implementation for User Story 2

- [X] T050 [US2] Create `website/src/components/profile/PersonalizationBar.tsx` component.
- [X] T051 [P] [US2] Create unit tests for `PersonalizationBar.tsx` in `website/tests/unit/components/profile/PersonalizationBar.test.ts`.
- [X] T052 [US2] Implement `PersonalizationBar.tsx` with click handler.
- [X] T053 [US2] Integrate `PersonalizationBar.tsx` into Docusaurus layout (`website/src/theme/Layout/index.tsx`).
- [X] T054 [US2] [P] Create E2E tests for the "Personalize" trigger in `tests/e2e/personalization_bar.spec.ts`.

---

## Final Phase: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and overall system quality.

- [X] T055 Implement backend Zod validation in `backend/src/api/profile/routes.py` for profile data.
- [X] T056 Implement frontend Zod validation integration in `website/src/pages/signup-wizard/index.tsx` and wizard steps.
- [X] T057 Implement clear error handling and user feedback for all API interactions (frontend `useAuth.ts`, `useProfile.ts`).
- [X] T058 Display privacy statement (`FR-006`) prominently during signup flow (`website/src/pages/signup-wizard/index.tsx`).
- [X] T059 Ensure graceful handling of missing profile data (`Edge Case`) in `website/src/hooks/useProfile.ts` and `PersonalizationBar.tsx`.
- [X] T060 Run `quickstart.md` validation.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately.
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories.
- **User Stories (Phase 3+)**: All depend on Foundational phase completion.
  - User stories can then proceed in parallel (if staffed).
  - Or sequentially in priority order (P1 → P2).
- **Polish (Final Phase)**: Depends on all desired user stories being complete.

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories.
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Integrates with US1's profile data but should be independently testable for UI visibility and click action.

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation.
- Models before services.
- Services before endpoints.
- Core implementation before integration.
- Story complete before moving to next priority.

### Parallel Opportunities

- All tasks marked [P] can run in parallel within their respective phases, provided they operate on different files and do not have incomplete task dependencies.
- Once the Foundational phase is complete, User Story 1 and parts of User Story 2 could theoretically be worked on in parallel by different team members (e.g., backend for US1, frontend for US1, then frontend for US2).

---

## Parallel Example: User Story 1

```bash
# Frontend components for Wizard (can be developed in parallel):
- [ ] T043 [P] [US1] Create `website/src/components/profile/ProfileWizardStep1.tsx` (Basics).
- [ ] T044 [P] [US1] Create `website/src/components/profile/ProfileWizardStep2.tsx` (Background).
- [ ] T045 [P] [US1] Create `website/src/components/profile/ProfileWizardStep3.tsx` (Strategy).

# Backend components for Profile (can be developed in parallel):
- [ ] T025 [US1] Create `backend/src/models/profile.py` (Pydantic model for User Profile).
- [ ] T029 [US1] Create `backend/src/services/profile_service.py` file.
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Backend + Frontend)
   - Developer B: User Story 2 (Frontend)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
