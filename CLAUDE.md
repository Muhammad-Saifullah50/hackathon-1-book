# Claude Code Rules — Physical AI & Humanoid Robotics Platform

> **Project**: Physical AI & Humanoid Robotics Learning Platform
> **Type**: Full-Stack Educational Monorepo (Docusaurus + FastAPI + RAG Tutor)
> **Version**: 1.0.0
> **Last Updated**: 2025-12-13

You are an expert AI assistant specializing in Spec-Driven Development (SDD) for building a world-class technical textbook platform on Physical AI and Humanoid Robotics.

---

## Project Overview

This is a **hybrid full-stack monorepo** containing:
- **Frontend**: Docusaurus 3.9 static site with React 19, TypeScript 5.6, and Tailwind CSS
- **Backend**: Python FastAPI with RAG-powered AI tutor agent (OpenAI + Qdrant)
- **Database**: Migrating from Supabase to Neon PostgreSQL + Better Auth

### Directory Structure
```
├── website/               # Docusaurus learning platform (React/TypeScript)
│   ├── src/
│   │   ├── components/    # React components (auth, profile, landing, ui)
│   │   ├── hooks/         # Custom React hooks (useAuth, useProfile)
│   │   ├── pages/         # Custom pages (signup, login, signup-wizard)
│   │   ├── types/         # TypeScript interfaces
│   │   └── theme/         # Docusaurus theme customizations
│   ├── docs/              # Markdown content (4 modules, 13 weeks)
│   └── tests/             # Unit tests (Jest) and E2E tests (Playwright)
├── backend/               # Python FastAPI backend
│   ├── src/api/           # API routes (auth, profile)
│   ├── src/services/      # Business logic layer
│   ├── src/models/        # Pydantic data models
│   └── tests/             # PyTest tests
├── specs/                 # Feature specifications (SDD)
├── history/               # PHRs and ADRs
├── .specify/              # SpecKit Plus templates
└── tests/e2e/             # Root-level E2E tests
```

---

## Technology Stack

### Frontend (website/)
| Technology | Version | Purpose |
|------------|---------|---------|
| Docusaurus | 3.9.2 | Static site generator |
| React | 19.0.0 | UI framework |
| TypeScript | 5.6.2 | Type safety |
| Tailwind CSS | 3.4.18 | Utility-first styling |
| Radix UI | 2.2.6+ | Accessible components |
| Framer Motion | 12.23.26 | Animations |
| Zod | 4.1.13 | Schema validation |
| Jest | 30.2.0 | Unit testing |
| Playwright | 1.57.0 | E2E testing |

### Backend (backend/)
| Technology | Purpose |
|------------|---------|
| FastAPI | Async Python web framework |
| uvicorn | ASGI server |
| openai-agents | OpenAI agent framework |
| qdrant-client | Vector database for RAG |
| pydantic | Data validation |
| pytest | Testing framework |

### Infrastructure
| Component | Current | Target |
|-----------|---------|--------|
| Database | Supabase PostgreSQL | Neon Serverless |
| Auth | Supabase Auth | Better Auth |
| Vector DB | Qdrant | Qdrant |
| Hosting | Local dev | TBD |

---

## Core Philosophy (from Constitution)

### Prime Directive
Author and engineer a world-class technical textbook titled "Physical AI & Humanoid Robotics" that bridges digital AI with embodied intelligence.

### Key Principles
1. **Physics First**: Always consider gravity, friction, and sensor noise in code
2. **Sim-to-Real**: Design → Simulation (Digital Twin) → Real Deployment
3. **Safety**: Emphasize safety protocols in all robotics code
4. **TDD Mandate**: Red-Green-Refactor cycle for all features
5. **Clarity**: Complex concepts (Kinematics, SLAM, VLA) explained step-by-step

---

## Development Commands

### Frontend (website/)
```bash
cd website
npm install          # Install dependencies
npm start            # Dev server on :3000
npm run build        # Production build
npm test             # Jest unit tests
npm run test:e2e     # Playwright E2E tests
```

### Backend (backend/)
```bash
cd backend
uv sync                                    # Install dependencies
uv run uvicorn main:app --reload          # Dev server on :8000
uv run pytest                             # Run tests
uv run python scripts/ingest_book.py      # Ingest content to Qdrant
```

---

## Task Context

**Your Surface:** Project-level guidance and development tasks via SDD.

**Success Metrics:**
- All outputs strictly follow user intent
- PHRs created automatically for every significant interaction
- ADR suggestions made for architecturally significant decisions
- Changes are small, testable, and reference code precisely

---

## Core Guarantees

### PHR (Prompt History Record) Creation
After completing requests, you **MUST** create a PHR.

**When to create PHRs:**
- Implementation work (code changes, features)
- Planning/architecture discussions
- Debugging sessions
- Spec/task/plan creation
- Multi-step workflows

**PHR Routing (under `history/prompts/`):**
- Constitution → `history/prompts/constitution/`
- Feature stages → `history/prompts/<feature-name>/`
- General → `history/prompts/general/`

**PHR Creation Process:**
1. Detect stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general
2. Generate 3-7 word title and slug
3. Read template from `.specify/templates/phr-template.prompt.md`
4. Fill ALL placeholders (ID, TITLE, STAGE, DATE_ISO, etc.)
5. Write file with appropriate routing
6. Validate: no unresolved placeholders, complete PROMPT_TEXT

### ADR (Architecture Decision Record) Suggestions
When architectural decisions are detected, suggest:
```
📋 Architectural decision detected: <brief>
   Document reasoning and tradeoffs? Run `/sp.adr <decision-title>`
```
Never auto-create ADRs; require user consent.

---

## Development Guidelines

### 1. Authoritative Source Mandate
Use MCP tools and CLI commands for all information gathering. NEVER assume solutions from internal knowledge.

### 2. Human as Tool Strategy
Invoke the user for input when encountering:
- **Ambiguous Requirements**: Ask 2-3 targeted clarifying questions
- **Unforeseen Dependencies**: Surface and ask for prioritization
- **Architectural Uncertainty**: Present options with tradeoffs
- **Completion Checkpoint**: Summarize and confirm next steps

### 3. Code Standards

**Frontend (TypeScript/React):**
- Use functional components with hooks
- Type everything with TypeScript interfaces
- Validate with Zod schemas
- Follow Tailwind CSS conventions
- Use Radix UI for accessible components
- Prefer `useSafeColorMode` over `useColorMode` for theme

**Backend (Python/FastAPI):**
- Full async/await patterns
- Pydantic models for validation
- Dependency injection with `Depends`
- Service layer for business logic
- Proper HTTPException error handling

**Testing:**
- Jest + React Testing Library for frontend unit tests
- Playwright for E2E tests (multi-browser)
- PyTest for backend tests
- Follow TDD: Red → Green → Refactor

### 4. Default Policies
- Clarify and plan first; separate business from technical understanding
- Never hardcode secrets; use `.env` files
- Prefer smallest viable diff; no unrelated refactoring
- Cite existing code with references (file:line)
- Keep reasoning private; output decisions and artifacts only

---

## Feature Specifications

### Active Features (specs/)
| ID | Feature | Status |
|----|---------|--------|
| 001 | Core Learning Experience | Active |
| 002 | Book Content Rules | Active |
| 003 | Module 1: Foundations | Active |
| 004 | Module 2: Digital Twin | Active |
| 005 | Module 3: AI-Robot Brain | Active |
| 006 | Module 4: VLA Models | Active |
| 007 | RAG Tutor Agent | Active |
| 008 | Learner Auth & Profile | Active |
| 009 | Dark Mode & Landing Page | Active |
| 010 | Neon Database Migration | **Current** |

### Current Branch Context
Branch `010-neon-database-migration` is migrating:
- Supabase Auth → Better Auth
- Supabase PostgreSQL → Neon Serverless
- Auth handled client-side with JWT validation in FastAPI

---

## Component Inventory

### Frontend Components
**Auth:** `AuthNavbarItems`, `LoginForm`, `SignupForm`
**Profile:** `ProfileWizardStep1/2/3`, `PersonalizationBar`
**Landing:** `HeroSection`, `FeaturesSection`, `LabSection`, `AnimatedSection`, `ChatbotDemo`, `PersonalizationDemo`
**UI:** `select`, `sheet`, `dialog`, `card`, `badge`, `alert`, `table`, `image`, `Logo`
**Theme:** `Heading`, `NavbarItem`, `CodeBlock`, `DocPaginator`, `DocSidebar`, `Layout`

### Backend Routes
- `POST /auth/signup` - User registration
- `POST /auth/login` - User login
- `POST /auth/logout` - User logout
- `GET /auth/me` - Current user info
- `POST /chatkit` - RAG tutor agent endpoint

---

## API Contracts

### Auth Endpoints (FastAPI)
```python
# POST /auth/signup
{
  "email": "string",
  "password": "string"
}
# Response: { "user": {...}, "session": {...} }

# POST /auth/login
{
  "email": "string",
  "password": "string"
}
# Response: { "access_token": "string", "refresh_token": "string" }
```

### Profile Schema (Zod)
```typescript
{
  age_range: "under_18" | "18_24" | "25_34" | "35_44" | "45_plus",
  education_level: "high_school" | "bachelors" | "masters" | "phd" | "self_taught",
  tech_background: "none" | "beginner" | "intermediate" | "advanced",
  primary_goal: "career" | "research" | "hobby" | "education",
  learning_mode: "visual" | "reading" | "hands_on" | "mixed",
  learning_speed: "thorough" | "balanced" | "accelerated",
  time_per_week: "1_3" | "4_7" | "8_15" | "16_plus",
  preferred_language: "en" | "es" | "zh" | "ar"
}
```

---

## Slash Commands

| Command | Purpose |
|---------|---------|
| `/sp.specify` | Create/update feature specification |
| `/sp.plan` | Generate implementation plan |
| `/sp.tasks` | Generate dependency-ordered tasks.md |
| `/sp.implement` | Execute implementation plan |
| `/sp.clarify` | Ask clarification questions for spec |
| `/sp.adr` | Create Architecture Decision Record |
| `/sp.phr` | Record Prompt History Record |
| `/sp.analyze` | Cross-artifact consistency analysis |
| `/sp.checklist` | Generate custom checklist |
| `/sp.git.commit_pr` | Git workflow automation |

---

## Execution Contract

For every request:
1. Confirm surface and success criteria (one sentence)
2. List constraints, invariants, non-goals
3. Produce artifact with acceptance checks inlined
4. Add follow-ups and risks (max 3 bullets)
5. Create PHR in appropriate subdirectory
6. Surface ADR suggestions for significant decisions

### Minimum Acceptance Criteria
- Clear, testable acceptance criteria included
- Explicit error paths and constraints stated
- Smallest viable change; no unrelated edits
- Code references to modified files (file:line format)

---

## Environment Configuration

### Required Environment Variables
**Backend (.env):**
```
OPENAI_API_KEY=sk-...
SUPABASE_URL=https://...
SUPABASE_KEY=eyJ...
QDRANT_URL=http://localhost:6333
```

**Frontend:**
- Backend URL defaults to `http://localhost:8000`
- Configurable via Docusaurus custom fields

### Runtime Requirements
- Node.js >= 20.0
- Python >= 3.12
- npm (frontend)
- uv (backend package manager)

---

## File References

| File | Purpose |
|------|---------|
| `website/docusaurus.config.ts` | Site configuration |
| `website/tailwind.config.js` | Tailwind theme |
| `website/jest.config.js` | Test configuration |
| `backend/main.py` | FastAPI app entry |
| `backend/pyproject.toml` | Python dependencies |
| `.specify/memory/constitution.md` | Project principles |
| `.specify/templates/` | SDD templates |

---

## Recent Changes
- 010-neon-database-migration: Added Python 3.12 (backend), TypeScript 5.x (frontend) + FastAPI, Pydantic, Better Auth, Docusaurus 3.x, python-jose

```
1180c5b feat: integrate Radix UI Select component and update profile wizard steps
c60c5b4 Add unit tests for landing components and hooks
007c77e corrected chatbot
8451a9e completed rag
e41f443 feat: created initial book structure
```

---

## Quick Reference

**Start Development:**
```bash
# Terminal 1: Frontend
cd website && npm start

# Terminal 2: Backend
cd backend && uv run uvicorn main:app --reload
```

**Run Tests:**
```bash
# Frontend unit tests
cd website && npm test

# Backend tests
cd backend && uv run pytest

# E2E tests
npm run test:e2e
```

**Create Feature Spec:**
```
/sp.specify <feature-description>
```

**Generate Tasks:**
```
/sp.tasks
```

## Active Technologies
- Python 3.12 (backend), TypeScript 5.x (frontend) + FastAPI, Pydantic, Better Auth, Docusaurus 3.x, python-jose (010-neon-database-migration)
- Neon Serverless PostgreSQL (replacing Supabase PostgreSQL) (010-neon-database-migration)
