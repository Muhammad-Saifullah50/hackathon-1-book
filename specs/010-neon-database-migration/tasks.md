# Tasks: Neon Database Migration with Better Auth

**Input**: Design documents from `/specs/010-neon-database-migration/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, data-model.md ✅, contracts/ ✅

**Tests**: TDD required per constitution - tests must be written first for JWT validation, DB connection, auth flows.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

**Deployment Model**: Separate deployments (Docusaurus static, Express auth-server, FastAPI backend)

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/src/`, `backend/tests/`
- **Frontend**: `website/src/`
- **Auth Server**: `auth-server/src/`
- **Scripts**: `scripts/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and structure for all three deployments

- [ ] T001 Create auth-server directory structure per plan.md (`auth-server/src/`, `auth-server/package.json`, `auth-server/tsconfig.json`)
- [ ] T002 [P] Initialize auth-server Node.js project with dependencies (express, better-auth, pg, dotenv) in `auth-server/package.json`
- [ ] T003 [P] Add backend Python dependencies (PyJWT[crypto], psycopg2-binary) to `backend/requirements.txt`
- [ ] T004 [P] Add frontend client dependencies (better-auth client) to `website/package.json`
- [ ] T005 [P] Create environment template files (`backend/.env.example`, `auth-server/.env.example`, `website/.env.example`)
- [ ] T006 [P] Configure TypeScript for auth-server in `auth-server/tsconfig.json`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Database Schema

- [ ] T007 Create Better Auth server configuration in `auth-server/src/auth.ts` with Neon connection
- [ ] T008 Create database connection module in `auth-server/src/db.ts` using pg Pool
- [ ] T009 Generate Better Auth schema and migrate to Neon using `npx @better-auth/cli generate && migrate`
- [ ] T010 Create UserProfile table migration script in `scripts/migrations/001_create_user_profile.sql`

### Auth Server Setup

- [ ] T011 Create Express server entry point in `auth-server/src/index.ts` with Better Auth handler
- [ ] T012 [P] Configure CORS in auth-server for frontend and backend origins in `auth-server/src/index.ts`
- [ ] T013 [P] Configure JWT plugin with RS256 and 7-day expiration in `auth-server/src/auth.ts`

### Backend Infrastructure

- [ ] T014 Create database connection service in `backend/src/services/database.py` with psycopg2
- [ ] T015 [P] Create JWT validation service in `backend/src/services/jwt_service.py` using PyJWT with PyJWKClient
- [ ] T016 [P] Create auth dependency for FastAPI in `backend/src/api/auth/dependencies.py`
- [ ] T017 Update CORS middleware in `backend/main.py` to allow auth-server and frontend origins

### Frontend Auth Client

- [ ] T018 Create Better Auth client in `website/src/lib/auth-client.ts` with jwtClient plugin
- [ ] T019 Swizzle Docusaurus Root component in `website/src/theme/Root.js` with AuthProvider
- [ ] T020 Create AuthContext for state management in `website/src/context/AuthContext.tsx`

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Seamless Database Connection (Priority: P1) 🎯 MVP

**Goal**: Application connects to Neon database seamlessly, existing functionality works

**Independent Test**: Verify backend connects to Neon and executes query; health check returns 200

### Tests for User Story 1

- [ ] T021 [P] [US1] Unit test for database connection in `backend/tests/unit/test_database.py`
- [ ] T022 [P] [US1] Integration test for health check endpoint in `backend/tests/integration/test_health.py`

### Implementation for User Story 1

- [ ] T023 [US1] Implement get_db_connection context manager in `backend/src/services/database.py`
- [ ] T024 [US1] Add health check endpoint with DB verification in `backend/main.py`
- [ ] T025 [US1] Add connection error handling and logging in `backend/src/services/database.py`
- [ ] T026 [US1] Configure connection timeout (10s) and SSL mode in database service

**Checkpoint**: Database connection verified, health endpoint returns `{"status": "ok", "database": "connected"}`

---

## Phase 4: User Story 2 - User Registration with Better Auth (Priority: P1)

**Goal**: New users can create accounts via email/password on Docusaurus

**Independent Test**: Submit registration form, verify user created in Neon `user` and `account` tables

### Tests for User Story 2

- [ ] T027 [P] [US2] Contract test for sign-up endpoint in `auth-server/tests/test_signup.ts`
- [ ] T028 [P] [US2] Integration test for registration flow in `website/tests/integration/test_registration.ts`

### Implementation for User Story 2

- [ ] T029 [US2] Configure email/password auth in `auth-server/src/auth.ts`
- [ ] T030 [US2] Create registration page component in `website/src/pages/register.tsx`
- [ ] T031 [US2] Add registration form with email, password, name fields in `website/src/pages/register.tsx`
- [ ] T032 [US2] Implement form submission using authClient.signUp in registration page
- [ ] T033 [US2] Add client-side validation and error display in registration page
- [ ] T034 [US2] Handle duplicate email error gracefully (no sensitive info exposed)

**Checkpoint**: User can register, user/account/session records exist in Neon

---

## Phase 5: User Story 3 - User Login and Session Management (Priority: P1)

**Goal**: Registered users can log in and receive session, can access protected resources

**Independent Test**: Log in with valid credentials, verify session token returned, logout invalidates session

### Tests for User Story 3

- [ ] T035 [P] [US3] Contract test for sign-in endpoint in `auth-server/tests/test_signin.ts`
- [ ] T036 [P] [US3] Contract test for sign-out endpoint in `auth-server/tests/test_signout.ts`
- [ ] T037 [P] [US3] Integration test for login/logout flow in `website/tests/integration/test_login.ts`

### Implementation for User Story 3

- [ ] T038 [US3] Create login page component in `website/src/pages/login.tsx`
- [ ] T039 [US3] Add login form with email and password fields in `website/src/pages/login.tsx`
- [ ] T040 [US3] Implement form submission using authClient.signIn in login page
- [ ] T041 [US3] Update AuthContext to track session state and provide getToken method
- [ ] T042 [US3] Add logout functionality using authClient.signOut
- [ ] T043 [US3] Handle invalid credentials error with appropriate message
- [ ] T044 [US3] Verify auth state persists across Docusaurus navigation

**Checkpoint**: User can login/logout, session managed correctly, auth state persists across navigation

---

## Phase 6: User Story 4 - JWT Token Validation in FastAPI (Priority: P1)

**Goal**: FastAPI validates JWT tokens from Better Auth for secure API authentication

**Independent Test**: Send requests with valid/invalid/expired/missing JWT tokens, verify correct 200/401 responses

### Tests for User Story 4

- [ ] T045 [P] [US4] Unit test for JWT validation service in `backend/tests/unit/test_jwt_service.py`
- [ ] T046 [P] [US4] Contract test for protected endpoint auth in `backend/tests/contract/test_auth_middleware.py`
- [ ] T047 [P] [US4] Integration test for JWT flow in `backend/tests/integration/test_jwt_validation.py`

### Implementation for User Story 4

- [ ] T048 [US4] Implement validate_jwt function with JWKS fetching in `backend/src/services/jwt_service.py`
- [ ] T049 [US4] Add JWKS caching with lru_cache in `backend/src/services/jwt_service.py`
- [ ] T050 [US4] Implement get_current_user FastAPI dependency in `backend/src/api/auth/dependencies.py`
- [ ] T051 [US4] Handle expired token with 401 response
- [ ] T052 [US4] Handle invalid/tampered token with 401 response
- [ ] T053 [US4] Handle missing token with 401 response
- [ ] T054 [US4] Configure JWKS_URL, JWT_ISSUER, JWT_AUDIENCE from environment

**Checkpoint**: FastAPI correctly validates JWT, returns 401 for invalid tokens, extracts user_id from valid tokens

---

## Phase 7: User Story 5 - User Profile Operations (Priority: P1)

**Goal**: Authenticated users can create, read, and update their profiles in Neon

**Independent Test**: Create profile, retrieve it, update fields, verify all changes persist

### Tests for User Story 5

- [ ] T055 [P] [US5] Contract test for GET /api/profile in `backend/tests/contract/test_profile_get.py`
- [ ] T056 [P] [US5] Contract test for POST /api/profile in `backend/tests/contract/test_profile_create.py`
- [ ] T057 [P] [US5] Contract test for PUT /api/profile in `backend/tests/contract/test_profile_update.py`
- [ ] T058 [P] [US5] Integration test for profile CRUD flow in `backend/tests/integration/test_profile.py`

### Implementation for User Story 5

- [ ] T059 [US5] Update UserProfile model with Better Auth user_id FK in `backend/src/models/profile.py`
- [ ] T060 [US5] Update ProfileService to use Neon connection in `backend/src/services/profile_service.py`
- [ ] T061 [US5] Implement GET /api/profile with JWT auth in `backend/src/api/profile/routes.py`
- [ ] T062 [US5] Implement POST /api/profile with JWT auth in `backend/src/api/profile/routes.py`
- [ ] T063 [US5] Implement PUT /api/profile with JWT auth in `backend/src/api/profile/routes.py`
- [ ] T064 [US5] Handle profile not found (404) for GET/PUT
- [ ] T065 [US5] Handle profile already exists (409) for POST
- [ ] T066 [US5] Validate profile enum fields match allowed values

**Checkpoint**: Full profile CRUD works with JWT authentication, all response codes match API spec

---

## Phase 8: User Story 6 - Data Migration from Supabase (Priority: P2)

**Goal**: Migrate existing user and profile data from Supabase to Neon with zero data loss

**Independent Test**: Export sample data from Supabase, import to Neon, verify 100% record match

### Tests for User Story 6

- [ ] T067 [P] [US6] Unit test for migration functions in `scripts/tests/test_migration.py`
- [ ] T068 [P] [US6] Integration test for full migration flow in `scripts/tests/test_migration_e2e.py`

### Implementation for User Story 6

- [ ] T069 [US6] Create migration script structure in `scripts/migrate_supabase_to_neon.py`
- [ ] T070 [US6] Implement profile export from Supabase in migration script
- [ ] T071 [US6] Implement user mapping (Supabase auth.users → Better Auth user) in migration script
- [ ] T072 [US6] Implement profile import to Neon with FK update in migration script
- [ ] T073 [US6] Add batching for large data sets (avoid memory issues) in migration script
- [ ] T074 [US6] Add verification step to compare source and destination record counts
- [ ] T075 [US6] Add error logging for failed record migrations
- [ ] T076 [US6] Create rollback capability (delete migrated records) in migration script

**Checkpoint**: Migration script runs successfully, 100% data integrity verified

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T077 [P] Add detailed logging for auth operations in `backend/src/services/jwt_service.py`
- [ ] T078 [P] Add detailed logging for profile operations in `backend/src/services/profile_service.py`
- [ ] T079 [P] Add request/response logging middleware in `backend/main.py`
- [ ] T080 Run quickstart.md validation (verify all setup steps work end-to-end)
- [ ] T081 [P] Document deployment configuration for auth-server in `auth-server/README.md`
- [ ] T082 [P] Document environment variables in `docs/configuration.md`
- [ ] T083 Security audit: verify no hardcoded secrets, proper error messages
- [ ] T084 Performance test: verify JWT validation <100ms, login <3s, registration <5s

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-8)**: All depend on Foundational phase completion
  - US1 (Database) → Can start immediately after Foundational
  - US2 (Registration) → Can start immediately after Foundational
  - US3 (Login) → Can start immediately after Foundational
  - US4 (JWT Validation) → Can start immediately after Foundational
  - US5 (Profile) → Depends on US4 (needs JWT validation working)
  - US6 (Migration) → Can start after US1 (needs Neon connection verified)
- **Polish (Phase 9)**: Depends on all P1 user stories being complete

### User Story Dependencies

```
Phase 2 (Foundational)
        │
        ├─────────────────────────────────────────────────┐
        │                                                 │
        ▼                                                 ▼
┌───────────────┐    ┌───────────────┐    ┌───────────────┐
│ US1: Database │    │ US2: Register │    │ US3: Login    │
│    (P1)       │    │    (P1)       │    │    (P1)       │
└───────┬───────┘    └───────────────┘    └───────────────┘
        │
        │            ┌───────────────┐
        │            │ US4: JWT      │
        │            │    (P1)       │
        │            └───────┬───────┘
        │                    │
        ▼                    ▼
┌───────────────┐    ┌───────────────┐
│ US6: Migration│    │ US5: Profile  │
│    (P2)       │    │    (P1)       │
└───────────────┘    └───────────────┘
```

### Within Each User Story

- Tests MUST be written and FAIL before implementation (TDD required)
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- US1, US2, US3, US4 can all start in parallel after Foundational
- All tests for a user story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: Foundational Phase

```bash
# After Setup complete, launch foundational tasks in parallel:
Task T012: "Configure CORS in auth-server"
Task T013: "Configure JWT plugin with RS256"
Task T014: "Create database connection service in backend"
Task T015: "Create JWT validation service in backend"
Task T016: "Create auth dependency for FastAPI"
```

## Parallel Example: User Story 5 (Profile)

```bash
# Launch all tests for US5 together:
Task T055: "Contract test for GET /api/profile"
Task T056: "Contract test for POST /api/profile"
Task T057: "Contract test for PUT /api/profile"
Task T058: "Integration test for profile CRUD flow"
```

---

## Implementation Strategy

### MVP First (User Stories 1-4 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: US1 - Database Connection
4. Complete Phase 4: US2 - Registration
5. Complete Phase 5: US3 - Login
6. Complete Phase 6: US4 - JWT Validation
7. **STOP and VALIDATE**: Test all P1 stories independently
8. Deploy/demo if ready (MVP!)

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add US1 (Database) → Test independently → ✓
3. Add US2 (Registration) → Test independently → ✓
4. Add US3 (Login) → Test independently → ✓
5. Add US4 (JWT) → Test independently → ✓ (MVP Complete!)
6. Add US5 (Profile) → Test independently → ✓
7. Add US6 (Migration) → Test independently → ✓
8. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: US1 (Database) → US6 (Migration)
   - Developer B: US2 (Registration) + US3 (Login)
   - Developer C: US4 (JWT) → US5 (Profile)
3. Stories complete and integrate independently

---

## Summary

| Phase | User Story | Priority | Task Count |
|-------|------------|----------|------------|
| 1 | Setup | - | 6 |
| 2 | Foundational | - | 14 |
| 3 | US1: Database Connection | P1 | 6 |
| 4 | US2: Registration | P1 | 8 |
| 5 | US3: Login/Session | P1 | 10 |
| 6 | US4: JWT Validation | P1 | 10 |
| 7 | US5: Profile Operations | P1 | 12 |
| 8 | US6: Data Migration | P2 | 10 |
| 9 | Polish | - | 8 |
| **Total** | | | **84** |

**MVP Scope**: Phases 1-6 (US1-US4) = 44 tasks
**Full Scope**: All phases = 84 tasks

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- TDD required: Write tests first, verify they fail, then implement
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Separate deployments: Docusaurus (static), auth-server (Express), backend (FastAPI)
