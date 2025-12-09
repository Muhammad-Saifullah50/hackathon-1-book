# Implementation Plan: Authentication & Detailed Profiling

**Branch**: `008-learner-auth-profile` | **Date**: 2025-12-09 | **Spec**: specs/008-learner-auth-profile/spec.md
**Input**: Feature specification from `/specs/008-learner-auth-profile/spec.md`

## Summary

Implementation of an Authentication and Detailed Learner Profiling system. This feature aims to create a 'Learner DNA' for each user through a multi-step signup wizard, allowing for granular customization of textbook content. It leverages `better-auth` for authentication, Supabase for data storage, `Shadcn UI` for the user interface, and `Zod` for schema validation.

## Technical Context

**Language/Version**: Python (backend), TypeScript/JavaScript (frontend)
**Primary Dependencies**: `better-auth` (authentication), `Supabase` (database), `Shadcn UI` (UI components), `Zod` (validation), `React` (frontend framework)
**Storage**: PostgreSQL (via Supabase)
**Testing**: Test-Driven Development (TDD) approach will be used for all new components.
**Target Platform**: Web application (Docusaurus frontend, Python backend)
**Project Type**: Web application
**Performance Goals**: User-facing profile wizard completion in under 2 minutes. Efficient data retrieval for content personalization.
**Constraints**:
- Must use `better-auth` for authentication.
- Must use `Supabase` as the primary database.
- Must use `Shadcn UI` for UI components.
- Must use `Zod` for schema validation.
- User data privacy explicitly guaranteed: data used *only* for tailoring textbook experience.
- Progressive profiling: "Skip for now" option for wizard.
**Scale/Scope**: Individual user profiles for personalized learning experience across the textbook.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Prime Directive**: ✅ This feature directly supports the textbook by enabling personalized content delivery, aligning with the goal of bridging digital AI and embodied intelligence through tailored learning paths.
- **Role & Identity**: ✅ Consistent with the expert professor and full-stack engineer roles, focusing on structured learning.
- **Core Philosophy ("Embodied Intelligence")**: ✅ Indirectly supports by adapting content based on "Learner DNA", enhancing the "adaptability" of the learning experience.
- **Pedagogical Standards**: ✅ Crucial for "Adaptability" by allowing content to be tailored to individual learner backgrounds and goals.
- **Visual & UI Standards**: ✅ `Shadcn UI` choice aligns with the aesthetic goal of clean, modern, and beautiful interface. The personalization bar will be elegantly integrated.
- **Operational Guidelines**: ✅ New code will adhere to Code Integrity and be developed with Test-Driven Development (TDD). User-Centric focus is central to personalization.

## Project Structure

### Documentation (this feature)

```text
specs/008-learner-auth-profile/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── api/
│   │   └── auth/           # Authentication endpoints (better-auth integration)
│   │   └── profile/        # User profile endpoints
│   ├── models/
│   │   └── profile.py      # Pydantic models for user profile
│   └── services/
│       └── auth_service.py # Business logic for authentication
│       └── profile_service.py # Business logic for user profiles
└── tests/
    └── unit/
        └── services/
        └── api/

website/
├── src/
│   ├── components/
│   │   └── auth/           # Login, Signup, Logout components
│   │   └── profile/        # Profile wizard steps, Personalization bar
│   ├── pages/
│   │   └── signup-wizard/  # Page for the multi-step signup process
│   ├── data/
│   │   └── profile-schema.ts # Zod schemas for frontend validation
│   └── hooks/
│       └── useAuth.ts      # Auth context/hooks
│       └── useProfile.ts   # Profile data context/hooks
└── tests/
    └── unit/
        └── components/
        └── pages/
```

**Structure Decision**: The existing `backend/` and `website/` directories will be extended. New directories within `src/`, `components/`, `pages/`, `data/`, `hooks/`, and `tests/` will be created to house the feature-specific code, following a modular and domain-driven design.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | | |
