# Implementation Plan: Chatbot Persistence with Neon Database

**Branch**: `013-chatbot-persistence` | **Date**: 2025-12-16 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/013-chatbot-persistence/spec.md`

## Summary

Migrate the RAG Tutor chatbot from file-based storage (`chatkit_store.json`) to Neon PostgreSQL database persistence. This involves implementing a new `NeonChatKitStore` class that implements the ChatKit `Store` interface with async database operations, creating database tables for threads and messages, and migrating existing data. Only authenticated users get persistent storage; anonymous users use in-memory session storage.

## Technical Context

**Language/Version**: Python 3.12 (backend), TypeScript 5.x (frontend)
**Primary Dependencies**: FastAPI, asyncpg, openai-chatkit, Pydantic (backend); React 19, Docusaurus 3.9 (frontend)
**Storage**: Neon PostgreSQL (serverless) with connection pooling via asyncpg
**Testing**: pytest (backend), Jest (frontend unit), Playwright (E2E)
**Target Platform**: Linux server (backend), Web browsers (frontend)
**Project Type**: Web application (backend + frontend monorepo)
**Performance Goals**: Load 100 messages in <2s, persist messages in <500ms, support 1000 concurrent users
**Constraints**: 32KB max message size, offline-capable with local queue, last-write-wins for conflicts
**Scale/Scope**: ~1000 users, ~100 messages per thread average, multiple threads per user

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| Physics First | N/A | Not applicable - this is a data persistence feature |
| Sim-to-Real | N/A | Not applicable - no hardware interaction |
| Safety | PASS | No safety-critical operations; standard CRUD with auth checks |
| Code Integrity | PASS | Modular design with ChatKit Store interface abstraction |
| TDD Mandate | PASS | Tests planned: unit tests for store methods, integration tests for DB operations, E2E for chat persistence |
| User-Centric | PASS | Seamless persistence improves learning continuity |
| Visual Standards | N/A | Backend-focused; minimal UI changes (pending indicator only) |

**Gate Result**: PASS - All applicable principles satisfied

## Project Structure

### Documentation (this feature)

```text
specs/013-chatbot-persistence/
├── plan.md              # This file
├── research.md          # Phase 0 output (already complete)
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output (API contracts)
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   └── chat.py              # NEW: Chat thread/message models
│   ├── services/
│   │   └── chat_store.py        # NEW: NeonChatKitStore implementation
│   └── api/
│       └── chat/
│           └── routes.py        # NEW: Chat persistence API routes (if needed)
├── scripts/
│   └── migrations/
│       └── 005_create_chat_tables.sql  # NEW: Chat tables migration
├── chatkit_store.py             # MODIFY: Replace with NeonChatKitStore
├── chatkit_server.py            # MODIFY: Use new store
└── tests/
    ├── unit/
    │   └── services/
    │       └── test_chat_store.py      # NEW: Store unit tests
    └── integration/
        └── api/
            └── test_chat_persistence.py # NEW: Integration tests

website/
├── src/
│   ├── components/
│   │   └── ChatWidget.tsx       # MODIFY: Add pending indicator
│   ├── hooks/
│   │   └── useChat.ts           # NEW: Chat persistence hook (if needed)
│   └── services/
│       └── chatService.ts       # MODIFY: Add offline queue logic
└── tests/
    └── unit/
        └── hooks/
            └── useChat.test.ts  # NEW: Chat hook tests
```

**Structure Decision**: Web application structure with existing backend/ and website/ directories. New files follow established patterns from previous features (personalization, translation).

## Complexity Tracking

> No constitution violations - all gates passed.

## Implementation Phases

### Phase 1: Database Schema & Migration (Backend)
1. Create SQL migration script for chat_threads and chat_thread_items tables
2. Run migration against Neon database
3. Create Pydantic models for chat entities

### Phase 2: NeonChatKitStore Implementation (Backend)
1. Implement `NeonChatKitStore` class extending ChatKit `Store` interface
2. Implement all required async methods with asyncpg
3. Add connection pooling for serverless Neon
4. Write unit tests for each store method

### Phase 3: Integration & Migration (Backend)
1. Update `chatkit_server.py` to use `NeonChatKitStore`
2. Create migration script for existing `chatkit_store.json` data
3. Write integration tests for full persistence flow
4. Add user_id context passing from auth middleware

### Phase 4: Frontend Enhancements (Website)
1. Add pending indicator for messages being saved
2. Implement local queue for offline support
3. Add retry logic for failed saves
4. Write frontend unit tests

### Phase 5: E2E Testing & Validation
1. Write Playwright E2E tests for chat persistence
2. Validate migration of existing data
3. Performance testing against success criteria

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Connection pool exhaustion | Low | High | Configure pool limits, add monitoring |
| Migration data loss | Low | Critical | Backup before migration, run in stages |
| Performance degradation | Medium | Medium | Index optimization, query profiling |
| Offline queue overflow | Low | Low | Limit queue size, clear on successful sync |
