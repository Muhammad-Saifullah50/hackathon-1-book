# Implementation Plan: Page Content Personalization

**Branch**: `011-personalize-page` | **Date**: 2025-12-14 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/011-personalize-page/spec.md`

## Summary

Implement a page content personalization feature that allows users to click a "Personalize Page" button and receive documentation content tailored to their learning profile. The system uses OpenAI Agents SDK with dynamic instructions built at runtime based on user profile attributes (tech_background, learning_mode, learning_speed, preferred_language, etc.). A hybrid caching strategy uses localStorage for instant access and server-side metadata for cross-device awareness with free re-personalization.

## Technical Context

**Language/Version**: Python 3.12 (backend), TypeScript 5.6 (frontend)
**Primary Dependencies**:
- Backend: FastAPI, openai-agents, pydantic, python-jose (JWT)
- Frontend: React 19, Docusaurus 3.9, Zod, Tailwind CSS
**Storage**: Neon PostgreSQL (personalization_history, personalization_quota tables), localStorage (client-side cache)
**Testing**: pytest (backend), Jest + React Testing Library (frontend unit), Playwright (E2E)
**Target Platform**: Web (Linux server for backend, browser for frontend)
**Project Type**: Web application (frontend + backend)
**Performance Goals**:
- Personalization completes within 15 seconds for typical pages (<5000 words)
- Toggle between original/personalized within 100ms (cached)
- 90% success rate for personalization requests
**Constraints**:
- 5 personalizations per user per day (quota resets midnight UTC)
- Server metadata storage <200 bytes per record (no content storage)
- Preserve 100% of code blocks, images, diagrams unchanged
**Scale/Scope**:
- Single feature addition to existing platform
- 2 new database tables
- 4 new API endpoints
- Enhanced PersonalizationBar component

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Evidence |
|-----------|--------|----------|
| **Physics First** | N/A | This is a content delivery feature, not robotics control |
| **Sim-to-Real** | N/A | No hardware interaction |
| **Safety** | PASS | No safety-critical operations; content personalization only |
| **Clarity** | PASS | Feature enables personalized explanations for different backgrounds |
| **Action-Oriented** | PASS | Preserves code blocks and practical examples unchanged |
| **Adaptability** | PASS | Core purpose is adapting content to learner backgrounds |
| **Clean UI** | PASS | Enhances existing PersonalizationBar; non-obstructive design |
| **TDD Mandate** | PASS | Tests will be written first per constitution requirement |
| **Code Integrity** | PASS | Modular architecture with clear separation of concerns |

**Gate Result**: PASS - All applicable principles satisfied.

## Project Structure

### Documentation (this feature)

```text
specs/011-personalize-page/
├── plan.md              # This file
├── spec.md              # Feature specification (completed)
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output (OpenAPI spec)
│   └── api-spec.yaml
├── checklists/          # Quality checklists
│   └── requirements.md
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── api/
│   │   ├── auth/                    # Existing auth dependencies
│   │   └── personalization/         # NEW: Personalization routes
│   │       ├── __init__.py
│   │       ├── routes.py            # API endpoints
│   │       └── dependencies.py      # Request validation
│   ├── models/
│   │   ├── profile.py               # Existing user profile model
│   │   └── personalization.py       # NEW: Request/Response models
│   ├── services/
│   │   ├── database.py              # Existing DB service
│   │   ├── profile_service.py       # Existing profile service
│   │   └── personalization_service.py # NEW: Personalization logic
│   └── agents/
│       └── personalization_agent.py # NEW: OpenAI Agent with dynamic instructions
├── tests/
│   ├── unit/
│   │   └── test_personalization_service.py
│   ├── integration/
│   │   └── test_personalization_api.py
│   └── contract/
│       └── test_personalization_contracts.py
└── scripts/
    └── migrations/
        └── 003_create_personalization_tables.sql # NEW

website/                             # Frontend (Docusaurus)
├── src/
│   ├── components/
│   │   └── profile/
│   │       └── PersonalizationBar.tsx  # MODIFY: Add full functionality
│   ├── hooks/
│   │   ├── useProfile.tsx              # Existing
│   │   └── usePersonalization.ts       # NEW: Personalization state management
│   ├── services/
│   │   └── personalizationService.ts   # NEW: API client + localStorage cache
│   └── utils/
│       └── hashUtils.ts                # NEW: Profile/content hashing
├── tests/
│   └── unit/
│       ├── components/
│       │   └── PersonalizationBar.test.tsx
│       └── hooks/
│           └── usePersonalization.test.ts
└── tests/e2e/
    └── personalization.spec.ts         # NEW: E2E tests
```

**Structure Decision**: Web application structure with backend (FastAPI) and frontend (Docusaurus/React). New personalization module added to existing backend API structure. Frontend enhances existing PersonalizationBar component with new hooks and services.

## Complexity Tracking

> No constitution violations requiring justification. Design follows existing patterns.

| Aspect | Decision | Rationale |
|--------|----------|-----------|
| Hybrid caching | localStorage + server metadata | Balances instant access with cross-device awareness without storing large content server-side |
| OpenAI Agents SDK | Dynamic instructions pattern | Allows runtime customization based on user profile without multiple agent definitions |
| Quota tracking | Server-side database | Prevents manipulation; enables cross-device enforcement |

## Phase 0: Research Requirements

Research topics to resolve before implementation:

1. **OpenAI Agents SDK**: Dynamic instructions pattern, RunContextWrapper usage, best practices for content transformation
2. **Markdown preservation**: Strategies to preserve code blocks, images while transforming prose
3. **Profile hashing**: SHA-256 implementation for profile change detection
4. **localStorage management**: LRU eviction strategies, storage limits

## Phase 1: Design Deliverables

1. **research.md**: Consolidated findings from Phase 0 research
2. **data-model.md**: Database schema for personalization_history and personalization_quota tables
3. **contracts/api-spec.yaml**: OpenAPI 3.0 specification for personalization endpoints
4. **quickstart.md**: Developer setup and testing guide
