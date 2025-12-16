# Tasks: Chatbot Persistence with Neon Database

**Input**: Design documents from `/specs/013-chatbot-persistence/`
**Prerequisites**: plan.md, spec.md, data-model.md, contracts/, research.md

**Tests**: Included per TDD mandate from constitution

**Organization**: Tasks are grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Database schema and core models shared across all user stories

- [x] T001 Create SQL migration script in backend/scripts/migrations/005_create_chat_tables.sql with chat_threads and chat_thread_items tables
- [x] T002 Run database migration against Neon PostgreSQL
- [x] T003 [P] Create ChatThread Pydantic model in backend/src/models/chat.py
- [x] T004 [P] Create ChatThreadItem Pydantic model in backend/src/models/chat.py

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core NeonChatKitStore implementation that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T005 Create NeonChatKitStore class skeleton in backend/src/services/chat_store.py extending ChatKit Store interface
- [x] T006 Implement connection pooling with asyncpg in backend/src/services/chat_store.py
- [x] T007 [P] Write unit test for generate_thread_id in backend/tests/unit/services/test_chat_store.py
- [x] T008 [P] Write unit test for generate_item_id in backend/tests/unit/services/test_chat_store.py
- [x] T009 Implement generate_thread_id method in backend/src/services/chat_store.py
- [x] T010 Implement generate_item_id method in backend/src/services/chat_store.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Retrieve Previous Conversations (Priority: P1) 🎯 MVP

**Goal**: Authenticated users can see their previous chat history when they open the chatbot

**Independent Test**: User creates a conversation, closes browser, reopens, and verifies previous messages are visible

### Tests for User Story 1

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [x] T011 [P] [US1] Write unit test for load_thread in backend/tests/unit/services/test_chat_store.py
- [x] T012 [P] [US1] Write unit test for load_thread_items in backend/tests/unit/services/test_chat_store.py
- [x] T013 [P] [US1] Write integration test for loading thread history in backend/tests/integration/api/test_chat_persistence.py

### Implementation for User Story 1

- [x] T014 [US1] Implement load_thread method in backend/src/services/chat_store.py
- [x] T015 [US1] Implement load_thread_items method with cursor-based pagination in backend/src/services/chat_store.py
- [x] T016 [US1] Implement load_threads method for listing user's threads in backend/src/services/chat_store.py
- [x] T017 [US1] Update chatkit_server.py to use NeonChatKitStore instead of MyChatKitStore
- [x] T018 [US1] Add user_id to context in chatkit_server.py from auth middleware
- [ ] T019 [US1] Verify tests pass for User Story 1

**Checkpoint**: User Story 1 is fully functional - users can retrieve their chat history

---

## Phase 4: User Story 2 - Save New Messages Automatically (Priority: P1) 🎯 MVP

**Goal**: Messages are automatically persisted to database when user sends or AI responds

**Independent Test**: Send a message, check database directly to confirm it was persisted

### Tests for User Story 2

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T020 [P] [US2] Write unit test for save_thread in backend/tests/unit/services/test_chat_store.py
- [ ] T021 [P] [US2] Write unit test for add_thread_item in backend/tests/unit/services/test_chat_store.py
- [ ] T022 [P] [US2] Write unit test for save_item in backend/tests/unit/services/test_chat_store.py
- [ ] T023 [P] [US2] Write integration test for message persistence flow in backend/tests/integration/api/test_chat_persistence.py

### Implementation for User Story 2

- [ ] T024 [US2] Implement save_thread method in backend/src/services/chat_store.py
- [ ] T025 [US2] Implement add_thread_item method in backend/src/services/chat_store.py
- [ ] T026 [US2] Implement save_item method in backend/src/services/chat_store.py
- [ ] T027 [US2] Implement load_item method in backend/src/services/chat_store.py
- [ ] T028 [US2] Add 32KB content size validation in backend/src/services/chat_store.py
- [ ] T029 [US2] Verify tests pass for User Story 2

**Checkpoint**: User Stories 1 AND 2 complete - full read/write persistence working

---

## Phase 5: User Story 3 - Manage Multiple Conversation Threads (Priority: P2)

**Goal**: Users can create new threads, switch between threads, and see thread list

**Independent Test**: Create multiple threads, switch between them, verify each maintains its own history

### Tests for User Story 3

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T030 [P] [US3] Write integration test for creating new thread in backend/tests/integration/api/test_chat_persistence.py
- [ ] T031 [P] [US3] Write integration test for listing threads in backend/tests/integration/api/test_chat_persistence.py
- [ ] T032 [P] [US3] Write integration test for switching threads in backend/tests/integration/api/test_chat_persistence.py

### Implementation for User Story 3

- [ ] T033 [P] [US3] Create GET /chat/threads endpoint in backend/src/api/chat/routes.py
- [ ] T034 [P] [US3] Create GET /chat/threads/{id} endpoint in backend/src/api/chat/routes.py
- [ ] T035 [US3] Register chat router in backend/main.py
- [ ] T036 [US3] Add thread list UI component in website/src/components/ChatWidget.tsx
- [ ] T037 [US3] Add "New Chat" button in website/src/components/ChatWidget.tsx
- [ ] T038 [US3] Implement thread switching logic in website/src/components/ChatWidget.tsx
- [ ] T039 [US3] Verify tests pass for User Story 3

**Checkpoint**: Users can manage multiple conversation threads

---

## Phase 6: User Story 4 - Delete Conversation History (Priority: P3)

**Goal**: Users can delete threads to clean up their history

**Independent Test**: Create a thread, delete it, verify it no longer appears

### Tests for User Story 4

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T040 [P] [US4] Write unit test for delete_thread in backend/tests/unit/services/test_chat_store.py
- [ ] T041 [P] [US4] Write unit test for delete_thread_item in backend/tests/unit/services/test_chat_store.py
- [ ] T042 [P] [US4] Write integration test for thread deletion in backend/tests/integration/api/test_chat_persistence.py

### Implementation for User Story 4

- [ ] T043 [US4] Implement delete_thread method in backend/src/services/chat_store.py
- [ ] T044 [US4] Implement delete_thread_item method in backend/src/services/chat_store.py
- [ ] T045 [US4] Create DELETE /chat/threads/{id} endpoint in backend/src/api/chat/routes.py
- [ ] T046 [US4] Add delete button to thread list UI in website/src/components/ChatWidget.tsx
- [ ] T047 [US4] Add confirmation dialog for deletion in website/src/components/ChatWidget.tsx
- [ ] T048 [US4] Verify tests pass for User Story 4

**Checkpoint**: All core user stories implemented

---

## Phase 7: Frontend Enhancements (Cross-Story)

**Purpose**: Offline resilience and UX improvements that span multiple stories

- [ ] T049 [P] Implement offline message queue in website/src/services/chatService.ts
- [ ] T050 [P] Add pending indicator component in website/src/components/ChatWidget.tsx
- [ ] T051 [P] Implement retry logic for failed saves in website/src/services/chatService.ts
- [ ] T052 [P] Write unit test for offline queue in website/tests/unit/services/chatService.test.ts
- [ ] T053 Add network status detection in website/src/hooks/useNetworkStatus.ts
- [ ] T054 Integrate offline queue with ChatWidget in website/src/components/ChatWidget.tsx

---

## Phase 8: Data Migration & Legacy Support

**Purpose**: Migrate existing chatkit_store.json data to database

- [ ] T055 Create migration script in backend/scripts/migrate_chatkit_to_neon.py
- [ ] T056 Backup existing chatkit_store.json before migration
- [ ] T057 Run migration script and validate data integrity
- [ ] T058 Remove or deprecate chatkit_store.json file-based implementation

---

## Phase 9: E2E Testing & Validation

**Purpose**: End-to-end validation of complete persistence flow

- [ ] T059 [P] Write E2E test for chat persistence across sessions in tests/e2e/chat_persistence.spec.ts
- [ ] T060 [P] Write E2E test for offline resilience in tests/e2e/chat_persistence.spec.ts
- [ ] T061 [P] Write E2E test for multi-device sync in tests/e2e/chat_persistence.spec.ts
- [ ] T062 Run performance tests against success criteria (<2s load, <500ms save)
- [ ] T063 Verify all quickstart.md test scenarios pass

---

## Phase 10: Polish & Documentation

**Purpose**: Final improvements and documentation

- [ ] T064 [P] Add code comments and docstrings in backend/src/services/chat_store.py
- [ ] T065 [P] Update README with chat persistence instructions
- [ ] T066 [P] Add monitoring/logging for database operations
- [ ] T067 Review and optimize database queries and indexes
- [ ] T068 Final validation of all user stories

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phases 3-6)**: All depend on Foundational phase completion
  - US1 and US2 can proceed in parallel (different methods in same class but independent)
  - US3 depends on US1 (needs load_threads working)
  - US4 is independent
- **Frontend Enhancements (Phase 7)**: Can start after US2 (save logic exists)
- **Migration (Phase 8)**: Should run after US1 and US2 complete
- **E2E Testing (Phase 9)**: Depends on all desired user stories being complete
- **Polish (Phase 10)**: Final phase after everything works

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 3 (P2)**: Depends on User Story 1 (needs load_threads to list threads)
- **User Story 4 (P3)**: Can start after Foundational (Phase 2) - Independent

### Within Each User Story

- Tests MUST be written and FAIL before implementation (TDD mandate)
- Unit tests → Implementation → Integration tests pass
- Backend before frontend (for stories involving UI)

### Parallel Opportunities

**Phase 1 (Setup)**: T003 and T004 can run in parallel (different models)

**Phase 2 (Foundational)**:
- T007 and T008 (tests) can run in parallel
- T009 and T010 (implementations) can run in parallel after their tests

**Phases 3 & 4 (US1 & US2)**: Can be worked on in parallel by different developers
- US1 tasks (T011-T019)
- US2 tasks (T020-T029)

**Phase 7 (Frontend)**: T049, T050, T051, T052 can all run in parallel (different files)

**Phase 9 (E2E)**: T059, T060, T061 can run in parallel (different test files)

**Phase 10 (Polish)**: T064, T065, T066 can run in parallel

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together:
Task: "Write unit test for load_thread in backend/tests/unit/services/test_chat_store.py"
Task: "Write unit test for load_thread_items in backend/tests/unit/services/test_chat_store.py"
Task: "Write integration test for loading thread history in backend/tests/integration/api/test_chat_persistence.py"

# After tests written, implement in sequence (same file):
Task: "Implement load_thread method"
Task: "Implement load_thread_items method"
Task: "Implement load_threads method"
```

---

## Parallel Example: User Story 2

```bash
# Launch all tests for User Story 2 together:
Task: "Write unit test for save_thread"
Task: "Write unit test for add_thread_item"
Task: "Write unit test for save_item"

# Implement methods (sequential - same file):
Task: "Implement save_thread"
Task: "Implement add_thread_item"
Task: "Implement save_item"
```

---

## Implementation Strategy

### MVP First (User Stories 1 & 2 Only)

Both User Story 1 and User Story 2 are marked P1 priority and together form the core persistence functionality:

1. Complete Phase 1: Setup (database schema)
2. Complete Phase 2: Foundational (NeonChatKitStore skeleton)
3. Complete Phase 3: User Story 1 (read operations)
4. Complete Phase 4: User Story 2 (write operations)
5. **STOP and VALIDATE**: Test both stories independently
6. Deploy/demo if ready

**This is the MVP** - users can save and retrieve chat history.

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 + 2 together → Test independently → **Deploy MVP!**
3. Add User Story 3 (thread management) → Test independently → Deploy
4. Add User Story 4 (deletion) → Test independently → Deploy
5. Add Frontend enhancements (offline support)
6. Run migration and E2E tests

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (read operations)
   - Developer B: User Story 2 (write operations)
   - Both can work in parallel on different methods in chat_store.py
3. Developer C: User Story 3 (thread UI - depends on US1)
4. Developer D: User Story 4 (deletion - independent)
5. Developer E: Frontend enhancements (Phase 7)

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing (TDD Red-Green-Refactor)
- US1 and US2 together form the MVP - both needed for full persistence
- Anonymous users use in-memory storage only (no DB tasks needed)
- Offline queue (Phase 7) handles database unavailability per clarifications
