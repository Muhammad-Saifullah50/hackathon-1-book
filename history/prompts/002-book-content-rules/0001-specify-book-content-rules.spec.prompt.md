---
id: 0001
title: specify-book-content-rules
stage: spec
date: 2025-12-08
surface: agent
model: gemini-2.0-flash-exp
feature: 002-book-content-rules
branch: 002-book-content-rules
user: saifullah
command: /sp.specify
labels: ["spec", "book-content", "rules"]
links:
  spec: specs/002-book-content-rules/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-book-content-rules/spec.md
  - specs/002-book-content-rules/checklists/requirements.md
tests: []
---

## Prompt

now i want to generate cvontent for the book. # Content Generation Rules (The "Beautiful Book" Standard) ## 1. Chapter Structure (Standard Operating Procedure) Every chapter file (.mdx) must strictly follow this template: 1. **Front Matter:** Title, ID, and "Experience Level" (Beginner/Advanced). 2. **Hero Section:** A brief, inspiring introduction to the concept (e.g., "Why does a robot need to see?"). 3. **Learning Objectives:** A bulleted list of what the student will build. 4. **The "Personalization" Trigger:** * *Mandatory Component:* Insert a <PersonalizationBar /> (Shadcn component) here. * *Function:* "Click here to tailor this chapter for [Hardware Engineers] or [Software Engineers]." 5. **Core Theory:** Explanation using analogies (e.g., "ROS 2 Topics are like a radio broadcast"). 6. **Interactive Diagram:** Use Mermaid.js or an embedded React component to visualize the concept. 7. **The Code Lab:** * Use Docusaurus <Tabs> to show code variations (e.g., "Python Agent" vs. "C++ Node"). * Always include a "Copy" button. 8. **The "Sim-to-Real" Warning:** * *Mandatory Component:* Use a Shadcn <Alert variant="destructive"> to highlight where simulation physics differ from reality (e.g., "In Gazebo, friction is perfect; in reality, your robot will slip."). ## 2. UI/UX Enforcement (Shadcn & Tailwind) * **Do not use standard Markdown tables.** Use Shadcn <Table> components for data. * **Do not use standard blockquotes.** Use Shadcn <Card> or <Callout> components for key takeaways. * **Typography:** Use Tailwind classes for spacing and fonts (e.g., className="prose prose-blue max-w-none"). * **Cleanliness:** Break up long text blocks with relevant icons or <Badge> tags. ## 3. Pedagogical Tone * **The "Professor" Persona:** You are authoritative but encouraging. * **"No Magic" Policy:** Never say "it just works." Explain *why* it works. * **Hardware Truth:** When discussing code, explicitly state if it runs on the **Workstation** (Training) or the **Jetson** (Edge). Keep these rules and not start writing anything

## Response snapshot

Created feature branch `002-book-content-rules` and specification file defining the 'Beautiful Book' content standards. Validated with checklist. All requirements from the input were mapped to functional requirements and success criteria.

## Outcome

- ✅ Impact: Established content standards for consistency.
- 🧪 Tests: n/a
- 📁 Files: Created spec and checklist.
- 🔁 Next prompts: /sp.plan
- 🧠 Reflection: Clear requirements made spec generation straightforward.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): none