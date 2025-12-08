---
id: 0001
title: specify-rag-tutor-agent
stage: spec
date: 2025-12-08
surface: agent
model: gemini-2.0-flash-exp
feature: 007-rag-tutor-agent
branch: 007-rag-tutor-agent
user: saifullah
command: /sp.specify
labels: ["spec", "rag-agent", "tutor", "user-stories"]
links:
  spec: specs/007-rag-tutor-agent/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/007-rag-tutor-agent/spec.md
  - specs/007-rag-tutor-agent/checklists/requirements.md
tests: []
---

## Prompt

# Feature Spec: The RAG Tutor Agent (User Stories) ## 1. Role & Identity * **Name:** "The Physical AI Tutor." * **Persona:** A helpful, patient, and rigorous teaching assistant. It does not just give answers; it guides the student to the solution using Socratic questioning when appropriate. * **Knowledge Boundary:** It answers questions based **ONLY** on the textbook content. If a user asks about unrelated topics (e.g., "knitting"), it politely declines and pivots back to Robotics. ## 2. Core Capabilities (User Stories) * **Story: Global Q&A** * As a student, I want to ask broad questions like "How does ROS 2 differ from ROS 1?" and receive a clear answer that cites specific chapters in the book. * **Story: "Ask about Selection" (Contextual Help)** * As a reader, I want to highlight a specific confusing paragraph (e.g., about "Quaternion Math"), click a floating "Explain This" button, and get a simplified explanation of *just that text* without losing my place. * **Story: Code Explanation** * As a developer, I want to paste a code snippet from the book into the chat and ask "What does this specific line do?", receiving a line-by-line breakdown of the syntax and logic. * **Story: Sim-to-Real Advice** * As a hardware engineer, I want to ask "Will this code work on my Jetson?" and get a specific warning if the code is meant only for simulation. ## 3. UI/UX Specifications (The "Beautiful" Interface) * **The Widget:** * A floating "Chat Bubble" icon in the bottom-right corner that is unobtrusive but always accessible. * When clicked, it opens a clean, modern side panel (Sheet) or floating window (Popover) with a smooth slide-in animation. * **Interaction Design:** * **User Messages:** Distinctly colored bubbles (matching the primary theme). * **Agent Messages:** Clean, neutral background with full Markdown support (to render code blocks, lists, and bold text beautifully). * **Feedback:** * A subtle "Thinking..." animation (like a pulsing skeleton loader) to indicate the agent is processing, so the interface never feels frozen. * **Citations:** * Every answer must include clickable links to the source material (e.g., "Read more in Module 1, Lesson 2") so I can verify the information. ## 4. Agent Skills (Functional Tools) * **Skill: `simplify_concept`** * The agent can reword complex physics jargon into plain English when I ask it to "Explain like I'm 5." * **Skill: `quiz_me`** * The agent can generate a 3-question multiple-choice quiz based on the chapter I am currently reading to test my understanding. * **Skill: `debug_helper`** * The agent can analyze error logs I paste (e.g., "Node not found") and suggest fixes based on the book's troubleshooting guides.

## Response snapshot

Created feature branch `007-rag-tutor-agent` and specification file defining the RAG Tutor Agent. The spec outlines the agent's persona, core capabilities (Global Q&A, Contextual Help, Code Explanation, Sim-to-Real Advice), UI/UX specifications, and specific agent skills. Two needs for clarification were identified regarding the text selection trigger implementation and the RAG backend architecture.

## Outcome

- ✅ Impact: Established a clear specification for the RAG Tutor Agent feature.
- 🧪 Tests: n/a
- 📁 Files: Created spec.md and requirements checklist.
- 🔁 Next prompts: /sp.plan
- 🧠 Reflection: The "Ask about Selection" feature adds a layer of frontend complexity that needs careful planning regarding browser API integration and state management.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): none