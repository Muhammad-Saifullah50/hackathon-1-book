---
id: 0011
title: fix-logo-branding-cleanup
stage: red
date: 2025-12-08
surface: agent
model: gemini-2.0-flash-exp
feature: 003-module-1-content
branch: 003-module-1-content
user: saifullah
command: /sp.implement
labels: ["fix", "logo", "branding", "cleanup", "docusaurus"]
links:
  spec: specs/003-module-1-content/spec.md
  plan: specs/003-module-1-content/plan.md
  tasks: specs/003-module-1-content/tasks.md
  ticket: null
  adr: null
  pr: null
files:
  - website/static/img/logo.svg
  - website/static/img/logo-dark.svg
  - website/docusaurus.config.ts
  - website/src/pages/index.tsx
tests: []
---

## Prompt

the logo is nopt visible in dark mode beczusae of its black color. change the button on the homeopage to link to /docs/intro. and remove all boilerplate code ansd pages

## Response snapshot

1.  **Fixed Logo Visibility**:
    *   Set `stroke="black"` for `website/static/img/logo.svg` (light mode).
    *   Created `website/static/img/logo-dark.svg` with `stroke="white"` (dark mode).
    *   Updated `website/docusaurus.config.ts` to use `srcDark: 'img/logo-dark.svg'` for the navbar logo in dark mode.
2.  **Updated Homepage Button**: Modified `website/src/pages/index.tsx` to change the button text to "Start the Book" and link it to `/docs/intro`.
3.  **Removed Boilerplate**:
    *   Deleted `website/docs/tutorial-basics` and `website/docs/tutorial-extras` directories.
    *   Deleted `website/src/pages/markdown-page.md`.

## Outcome

- ✅ Impact: Improved user experience with visible logo and streamlined navigation. Removed unnecessary boilerplate.
- 🧪 Tests: n/a
- 📁 Files: Modified 4 files, created 1 file, deleted 3 directories/files.
- 🔁 Next prompts: /sp.git.commit_pr
- 🧠 Reflection: Managing assets for light/dark mode requires careful configuration, especially with SVGs and Docusaurus.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): none