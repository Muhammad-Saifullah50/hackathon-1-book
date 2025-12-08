---
id: 0009
title: remove-blog-update-branding
stage: red
date: 2025-12-08
surface: agent
model: gemini-2.0-flash-exp
feature: 003-module-1-content
branch: 003-module-1-content
user: saifullah
command: /sp.implement
labels: ["cleanup", "branding", "docusaurus", "config"]
links:
  spec: specs/003-module-1-content/spec.md
  plan: specs/003-module-1-content/plan.md
  tasks: specs/003-module-1-content/tasks.md
  ticket: null
  adr: null
  pr: null
files:
  - website/docusaurus.config.ts
  - website/static/img/logo.svg
tests: []
---

## Prompt

now remove the blog sections in the app. also give the app a name and put it on its place. also put a rewlevant lucide icon in place of the placeholder icons

## Response snapshot

1.  **Removed Blog**: Disabled the blog plugin in `website/docusaurus.config.ts` by setting `blog: false` and deleting the `website/blog` directory.
2.  **Updated Branding**: Changed the site title to "Agentic AI" and tagline to "Embodied Intelligence". Updated navbar and footer labels to reflect "Book" instead of "Tutorial" or "Blog".
3.  **Updated Logo**: Replaced `website/static/img/logo.svg` with the `bot` icon from Lucide icons (SVG format).

## Outcome

- ✅ Impact: Cleaned up unused features and professionalized the branding.
- 🧪 Tests: n/a
- 📁 Files: Modified config and logo, deleted blog folder.
- 🔁 Next prompts: /sp.git.commit_pr
- 🧠 Reflection: Disabling the blog in Docusaurus requires both config change (`blog: false`) and optionally removing the folder/nav links to be thorough.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): none