---
id: 0005
title: fix-yaml-module-1
stage: red
date: 2025-12-08
surface: agent
model: gemini-2.0-flash-exp
feature: 003-module-1-content
branch: 003-module-1-content
user: saifullah
command: /sp.implement
labels: ["fix", "yaml", "docusaurus"]
links:
  spec: specs/003-module-1-content/spec.md
  plan: specs/003-module-1-content/plan.md
  tasks: specs/003-module-1-content/tasks.md
  ticket: null
  adr: null
  pr: null
files:
  - website/docs/module-01/04-capstone-project.mdx
tests: []
---

## Prompt

[ERROR] Error: Can't process doc metadata for doc at path path=/home/saifullah/projects/agentic-ai/hackathon-1-book/website/docs/module-01/04-capstone-project.mdx in version name=current
...
[cause]: YAMLException: incomplete explicit mapping pair; a key node is missed; or followed by a non-tabulated empty line at line 2, column 16:
      title: Capstone: The Blind Walker

## Response snapshot

Fixed YAML front matter syntax error in `website/docs/module-01/04-capstone-project.mdx` by quoting the title string containing a colon. Verified other files do not have the same issue.

## Outcome

- ✅ Impact: Fixed build error caused by invalid YAML.
- 🧪 Tests: n/a
- 📁 Files: Modified 1 file.
- 🔁 Next prompts: /sp.git.commit_pr
- 🧠 Reflection: YAML parsers are strict about colons in values; always quote strings with special characters.

## Evaluation notes (flywheel)

- Failure modes observed: yaml parsing error
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): none