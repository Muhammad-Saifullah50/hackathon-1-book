# Implementation Plan: Neon Database Migration with Better Auth

**Branch**: `010-neon-database-migration` | **Date**: 2025-12-13 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/010-neon-database-migration/spec.md`

## Summary

Migrate the backend infrastructure from Supabase to Neon Serverless Postgres and replace Supabase Auth with Better Auth. The architecture separates concerns: Docusaurus frontend hosts Better Auth for authentication (issuing JWTs), FastAPI backend validates JWTs and handles business logic, and Neon stores all data (auth tables + profiles).

## Technical Context

**Language/Version**: Python 3.12 (backend), TypeScript 5.x (frontend)
**Primary Dependencies**: FastAPI, Pydantic, Better Auth, Docusaurus 3.x, python-jose
**Storage**: Neon Serverless PostgreSQL (replacing Supabase PostgreSQL)
**Testing**: pytest (backend), Jest/Vitest (frontend)
**Target Platform**: Linux server (backend), Web (frontend)
**Project Type**: web (frontend + backend separation)
**Performance Goals**: JWT validation <100ms, login <3s, registration <5s, 50 concurrent DB ops
**Constraints**: 7-day JWT expiration with session refresh, RS256 algorithm, SSL/TLS required
**Scale/Scope**: Single learning platform, existing profile data migration required

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Evidence |
|-----------|--------|----------|
| **Prime Directive** | ✅ PASS | Infrastructure migration supports the Physical AI & Robotics textbook platform |
| **Sim-to-Real** | ✅ N/A | Not applicable - infrastructure feature, not robotics code |
| **Safety** | ✅ PASS | JWT-based auth, SSL/TLS encryption, no hardcoded credentials |
| **Code Integrity** | ✅ PASS | Modular services (AuthService, ProfileService), well-commented existing code |
| **Test-Driven Development** | ✅ REQUIRED | Tests MUST be written first for JWT validation, DB connection, auth flows |
| **Visual & UI Standards** | ✅ N/A | Backend infrastructure - UI changes minimal (auth pages only) |

**Gate Result**: ✅ PASS - Proceed to Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/010-neon-database-migration/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Web application structure (frontend + backend)
backend/
├── main.py                      # FastAPI app entry point
├── src/
│   ├── api/
│   │   ├── auth/routes.py       # Auth endpoints (to be updated for JWT validation)
│   │   └── profile/routes.py    # Profile CRUD endpoints
│   ├── models/
│   │   ├── auth.py              # User model (to be updated for Better Auth)
│   │   └── profile.py           # UserProfile model (existing)
│   └── services/
│       ├── auth_service.py      # Auth service (to be replaced - currently Supabase)
│       ├── profile_service.py   # Profile service (to be updated for Neon)
│       └── jwt_service.py       # NEW: JWT validation service
├── tests/
│   ├── unit/                    # Unit tests
│   ├── integration/             # Integration tests
│   └── contract/                # Contract tests for API
└── requirements.txt             # Python dependencies

website/
├── docusaurus.config.ts         # Docusaurus configuration
├── src/
│   ├── theme/
│   │   └── Root.js              # NEW: Better Auth provider wrapper (swizzled)
│   ├── pages/
│   │   ├── login.tsx            # NEW: Login page
│   │   └── register.tsx         # NEW: Registration page
│   ├── lib/
│   │   └── auth-client.ts       # NEW: Better Auth client configuration
│   └── components/              # Existing components
└── package.json                 # Node dependencies (add better-auth/client)

auth-server/                     # NEW: Companion Express server for Better Auth
├── src/
│   ├── index.ts                 # Express app entry point
│   ├── auth.ts                  # Better Auth server configuration
│   └── db.ts                    # Neon database connection (pg Pool)
├── package.json                 # Dependencies: better-auth, express, pg
└── tsconfig.json                # TypeScript config
```

**Structure Decision**: Web application pattern selected. Backend (FastAPI) and frontend (Docusaurus) are separate deployments. Better Auth API routes will be added to the Docusaurus project via a companion API server or serverless functions.

## Complexity Tracking

> No complexity violations detected. All decisions follow minimal viable patterns.

| Aspect | Decision | Rationale |
|--------|----------|-----------|
| Auth hosting | Co-located with Docusaurus | Better Auth is TypeScript-native; avoids Node.js in Python backend |
| JWT validation | python-jose with JWKS caching | Standard library, well-maintained, supports RS256 |
| Database driver | psycopg2-binary | Compatible with Neon, simpler than asyncpg for initial migration |
| User-Profile link | Foreign key (1:1) | Standard relational pattern, enforces referential integrity |
