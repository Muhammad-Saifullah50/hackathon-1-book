# Implementation Plan: Urdu Translation Bar

**Branch**: `012-urdu-translation` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/012-urdu-translation/spec.md`

## Summary

Implement a translation bar feature that allows users to click a "Translate to Urdu" button on any documentation page and receive the content translated into Urdu. The system uses OpenAI Agents SDK with a dedicated `urdu_translation_agent` to perform translations while preserving code blocks, images, and markdown structure. The feature follows the same rendering pattern as the personalization feature (dual-container approach) and includes quota management (5 translations/day per user or IP) with localStorage caching (30-day expiry).

## Technical Context

**Language/Version**: Python 3.12 (backend), TypeScript 5.6 (frontend)
**Primary Dependencies**:
- Backend: FastAPI, openai-agents, pydantic, python-jose (JWT - optional for auth users)
- Frontend: React 19, Docusaurus 3.9, Zod, Tailwind CSS, Lucide Icons
**Storage**: Neon PostgreSQL (translation_history, translation_quota tables), localStorage (client-side cache)
**Testing**: pytest (backend), Jest + React Testing Library (frontend unit), Playwright (E2E)
**Target Platform**: Web (Linux server for backend, browser for frontend)
**Project Type**: Web application (frontend + backend)
**Performance Goals**:
- Translation completes within 30 seconds for typical pages (<5000 words)
- Toggle between original/translated within 100ms (cached)
- 90% success rate for translation requests
**Constraints**:
- 5 translations per user/IP per day (quota resets midnight UTC)
- 30-day cache expiration for translated content
- Preserve 100% of code blocks, images, diagrams unchanged
- RTL text direction for Urdu content
**Scale/Scope**:
- Single feature addition to existing platform
- 2 new database tables (parallel to personalization)
- 3 new API endpoints
- New TranslationBar component + useTranslation hook

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Evidence |
|-----------|--------|----------|
| **Physics First** | N/A | This is a content delivery feature, not robotics control |
| **Sim-to-Real** | N/A | No hardware interaction |
| **Safety** | PASS | No safety-critical operations; content translation only |
| **Clarity** | PASS | Feature enables Urdu speakers to understand content in native language |
| **Action-Oriented** | PASS | Preserves code blocks and practical examples unchanged |
| **Adaptability** | PASS | Core purpose is making content accessible to Urdu speakers |
| **Clean UI** | PASS | TranslationBar follows same design system as PersonalizationBar |
| **TDD Mandate** | PASS | Tests will be written first per constitution requirement |
| **Code Integrity** | PASS | Modular architecture reusing personalization patterns |

**Gate Result**: PASS - All applicable principles satisfied.

## Project Structure

### Documentation (this feature)

```text
specs/012-urdu-translation/
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
│   │   ├── personalization/         # Existing (reference pattern)
│   │   └── translation/             # NEW: Translation routes
│   │       ├── __init__.py
│   │       ├── routes.py            # API endpoints
│   │       └── dependencies.py      # Request validation, IP extraction
│   ├── models/
│   │   ├── personalization.py       # Existing (reference pattern)
│   │   └── translation.py           # NEW: Request/Response models
│   ├── services/
│   │   ├── personalization_service.py # Existing (reference pattern)
│   │   └── translation_service.py   # NEW: Translation logic + quota
│   └── ai_agents/
│       ├── personalization_agent.py # Existing
│       └── translation_agent.py     # NEW: urdu_translation_agent
├── tests/
│   ├── unit/
│   │   └── services/
│   │       └── test_translation_service.py
│   ├── integration/
│   │   └── api/
│   │       └── test_translation_api.py
│   └── contract/
│       └── test_translation_contracts.py
└── scripts/
    └── migrations/
        └── 004_create_translation_tables.sql # NEW

website/                             # Frontend (Docusaurus)
├── src/
│   ├── components/
│   │   └── translation/
│   │       └── TranslationBar.tsx   # NEW: Translation UI component
│   ├── hooks/
│   │   ├── usePersonalization.ts    # Existing (reference pattern)
│   │   └── useTranslation.ts        # NEW: Translation state management
│   ├── services/
│   │   ├── personalizationService.ts # Existing (reference pattern)
│   │   └── translationService.ts    # NEW: API client + localStorage cache
│   ├── types/
│   │   └── translation.ts           # NEW: TypeScript interfaces
│   └── theme/
│       └── Heading/
│           └── index.tsx            # MODIFY: Add TranslationBar below PersonalizationBar
├── tests/
│   └── unit/
│       ├── components/
│       │   └── translation/
│       │       └── TranslationBar.test.tsx
│       └── hooks/
│           └── useTranslation.test.ts
└── tests/e2e/
    └── translation.spec.ts          # NEW: E2E tests
```

**Structure Decision**: Web application structure with backend (FastAPI) and frontend (Docusaurus/React). New translation module parallels existing personalization module structure. TranslationBar is a new component (not modifying PersonalizationBar) that renders below it in the Heading wrapper.

## Complexity Tracking

> No constitution violations requiring justification. Design follows existing personalization patterns.

| Aspect | Decision | Rationale |
|--------|----------|-----------|
| Separate TranslationBar | New component, not merged with PersonalizationBar | Separation of concerns; translation available to all users, personalization requires profile |
| Dual quota tracking | User ID for auth users, IP for anonymous | Mirrors personalization pattern; prevents abuse while maintaining accessibility |
| Same rendering pattern | Dual-container approach from personalization | Proven pattern; consistent UX; code reuse |
| RTL support | CSS direction property on translated container | Standard approach for RTL languages |

## Phase 0: Research Requirements

Research topics to resolve before implementation:

1. **OpenAI Agents SDK**: Translation-specific prompting, preserving markdown structure, handling technical terms
2. **RTL/LTR mixing**: CSS strategies for Urdu (RTL) text with embedded LTR code blocks
3. **IP-based quota tracking**: Reliable IP extraction from requests (X-Forwarded-For handling)
4. **30-day cache expiry**: localStorage timestamp comparison, LRU eviction with expiry

## Phase 1: Design Deliverables

1. **research.md**: Consolidated findings from Phase 0 research
2. **data-model.md**: Database schema for translation_history and translation_quota tables
3. **contracts/api-spec.yaml**: OpenAPI 3.0 specification for translation endpoints
4. **quickstart.md**: Developer setup and testing guide
