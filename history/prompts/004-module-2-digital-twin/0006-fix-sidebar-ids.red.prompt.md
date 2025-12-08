---
id: 0006
title: fix-sidebar-ids
stage: red
date: 2025-12-08
surface: agent
model: gemini-2.0-flash-exp
feature: 004-module-2-digital-twin
branch: 004-module-2-digital-twin
user: saifullah
command: /sp.implement
labels: ["fix", "docusaurus", "sidebar"]
links:
  spec: specs/004-module-2-digital-twin/spec.md
  plan: specs/004-module-2-digital-twin/plan.md
  tasks: specs/004-module-2-digital-twin/tasks.md
  ticket: null
  adr: null
  pr: null
files:
  - website/src/data/curriculum.json
tests: []
---

## Prompt

getting this errro Warn: `blogDir` doesn't exist: "/home/saifullah/projects/agentic-ai/hackathon-1-book/website/blog".
[ERROR] Error: Invalid sidebar file at "sidebars.ts".
These sidebar document ids do not exist:
- module-02/01-gazebo-physics
- module-02/02-unity-integration
- module-02/03-capstone-digital-twin

Available document ids are:
- module-02/capstone-digital-twin
- module-02/gazebo-physics
- module-02/overview
- module-02/unity-integration
...

## Response snapshot

Fixed the `Invalid sidebar file` error by updating `website/src/data/curriculum.json`. The `slug` values for Module 2 lessons were adjusted to match Docusaurus's default document ID generation, which removes numerical prefixes from filenames (e.g., `01-gazebo-physics` becomes `gazebo-physics`). This aligns the expected sidebar IDs with the available document IDs.

## Outcome

- ✅ Impact: Resolved Docusaurus build error related to sidebar configuration and document ID mismatch.
- 🧪 Tests: n/a
- 📁 Files: Modified 1 file.
- 🔁 Next prompts: /sp.git.commit_pr
- 🧠 Reflection: Docusaurus's automatic document ID generation needs to be considered when defining slugs in dynamic sidebar data sources like `curriculum.json`.

## Evaluation notes (flywheel)

- Failure modes observed: Invalid sidebar document IDs.
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): none