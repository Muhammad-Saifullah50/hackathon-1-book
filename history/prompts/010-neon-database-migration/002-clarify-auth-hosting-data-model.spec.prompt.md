---
id: 002
title: Clarify Auth Hosting and Data Model
stage: spec
date: 2025-12-13
surface: agent
model: claude-sonnet-4-5-20250929
feature: 010-neon-database-migration
branch: 010-neon-database-migration
user: saifullah
command: /sp.clarify
labels: ["clarification", "better-auth", "architecture", "jwt", "data-model"]
links:
  spec: specs/010-neon-database-migration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/010-neon-database-migration/spec.md
tests:
  - N/A (clarification phase)
---

## Prompt

/sp.clarify

User clarifications during session:
1. Q: Where will Better Auth API routes be hosted?
   Initial answer: "the auth routes will be hosted as the same way as other python api routes are hosted, in the same place"
   Refined answer after discussion: "better auth is a non negotiable requirement. i think it should be hosted on the same place as the frontend"
   Final: Co-located with Docusaurus frontend

2. Q: How are User and UserProfile related?
   Answer: "A" (Option A: UserProfile.user_id is a foreign key to Better Auth User.id)

3. Q: What is the JWT token expiration duration?
   Answer: "option b" (Medium-lived 7 days with session refresh)

## Response snapshot

Performed structured ambiguity scan across 10 taxonomy categories. Identified 3 high-impact clarification questions:

1. **Better Auth Hosting**: Clarified that Better Auth runs alongside Docusaurus frontend (not FastAPI). This is architecturally correct since Better Auth is TypeScript-based and integrates naturally with React/Node.js environments.

2. **User-Profile Relationship**: Confirmed 1:1 FK relationship between UserProfile.user_id and Better Auth User.id. Updated Key Entities section.

3. **JWT Expiration**: Set to 7-day expiration with session refresh. Updated FR-013 with specific configuration.

All 3 questions answered and integrated into spec. Created Clarifications section with Session 2025-12-13 subsection.

## Outcome

- ✅ Impact: Resolved 3 architectural ambiguities - auth hosting, data model relationships, session duration
- 🧪 Tests: N/A (clarification phase)
- 📁 Files: 1 file modified (spec.md - 5 sections updated)
- 🔁 Next prompts: `/sp.plan` to generate implementation architecture
- 🧠 Reflection: Critical clarification about Better Auth hosting prevented incorrect architecture (Python-based auth vs proper TypeScript integration)

## Evaluation notes (flywheel)

- Failure modes observed: Initial misunderstanding about auth hosting location - corrected after user feedback
- Graders run and results (PASS/FAIL): Manual validation - all clarifications properly integrated
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to planning phase to validate architectural decisions
