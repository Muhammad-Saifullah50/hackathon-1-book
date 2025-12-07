---
id: 0002
title: constitution-add-tdd
stage: constitution
date: 2025-12-07
surface: agent
model: Gemini
feature: none
branch: 001-core-learning-experience
user: saifullah
command: /sp.constitution
labels: ["governance", "TDD", "standards"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - (none)
---

## Prompt

use test driven development. for every feature write tests foirst and then write code to satiusfy thiose tests

## Response snapshot

Updated Constitution to v1.1.0. Added explicit 'Test-Driven Development (TDD)' mandate to Operational Guidelines (Section 6), requiring Red-Green-Refactor cycle.

## Outcome

- ✅ Impact: Constitution mandates TDD.
- 🧪 Tests: N/A
- 📁 Files: Updated constitution.md
- 🔁 Next prompts: /sp.plan
- 🧠 Reflection: User reinforced engineering discipline; reflected immediately in governance doc.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Pass
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): N/A