---
id: "0001"
title: "Create Dark Mode Landing Page Spec"
stage: spec
date: 2025-12-12
surface: agent
model: claude-sonnet-4-5-20250929
feature: 009-dark-mode-landing-page
branch: 009-dark-mode-landing-page
user: saifullah
command: /sp.specify
labels: ["landing-page", "dark-mode", "ui-spec", "physical-ai"]
links:
  spec: specs/009-dark-mode-landing-page/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/009-dark-mode-landing-page/spec.md
  - specs/009-dark-mode-landing-page/checklists/requirements.md
tests:
  - N/A (specification phase)
---

## Prompt

build the landing page for this book. from these specifications # Specification: The "Dark Mode" Landing Page
**Target File:** `src/pages/index.tsx` (Docusaurus)

## 1. Visual Theme & Identity
* **Concept:** "NASA meets Cyberpunk." Serious, futuristic, and high-precision.
* **Color Palette:**
    * **Background:** Deep Void (`bg-slate-950` or `#020617`).
    * **Text:** High-contrast White (`text-slate-50`) with muted grey subtitles (`text-slate-400`).
    * **Primary Accent:** Electric Cyan (`#22d3ee`) for key actions.
    * **Secondary Accent:** Deep Violet (`#7c3aed`) for gradients and depth.
* **Effects:**
    * **Glassmorphism:** All cards must use semi-transparent backgrounds with thin white borders (`bg-slate-900/50 backdrop-blur-md border border-white/10`).
    * **Glow:** Buttons and Hero text should have a subtle drop-shadow glow.

## 2. Technical Implementation
* **Framework:** React (Docusaurus).
* **Styling:** Tailwind CSS.
* **Animation:** **Framer Motion** (library).
    * *Rule:* Use `initial={{ opacity: 0, y: 20 }}` and `whileInView={{ opacity: 1, y: 0 }}` for all sections to create a smooth "fade-in-up" effect as the user scrolls.

## 3. Section Breakdown

### Section 1: The Hero ("The Awakening")
* **Layout:** Centered or Split (Text Left, Visual Right).
* **Headline:** Large, bold typography (text-6xl).
    * Text: **"Wake Up the Metal."**
    * Effect: Apply a gradient text fill (Cyan to Violet) on "Metal."
* **Subheadline:** "From Python code to Walking Robots. The comprehensive guide to Physical AI, ROS 2, and Humanoid Mechanics."
* **Visual:** A high-quality image of the **Unitree G1** or a wireframe robot hand reaching out from the darkness.
* **CTAs:**
    * Primary: **"Start Reading"** (Solid Cyan background, Black text, high visibility).
    * Secondary: **"Watch Demo"** (Transparent background, White border).

### Section 2: The Curriculum (Timeline Cards)
* **Title:** "The Journey to Embodied Intelligence."
* **Component:** A grid of 4 "Glass" Cards.
* **Content:**
    1.  **The Nervous System:** "Weeks 1-5: ROS 2 Middleware & Nodes." (Icon: Network Graph).
    2.  **The Digital Twin:** "Weeks 6-7: Simulation in Gazebo & Unity." (Icon: 3D Cube).
    3.  **The Brain:** "Weeks 8-10: NVIDIA Isaac & Perception." (Icon: AI Chip).
    4.  **The Body:** "Weeks 11-13: VLA & Humanoid Control." (Icon: Robot Walking).

### Section 3: The Features (Interactive Demos)
* **Layout:** Zig-Zag (Text Left/Image Right, then swap).
* **Feature A: The AI Tutor**
    * *Text:* "Never Get Stuck. Ask the RAG Chatbot."
    * *Visual:* An animated mock chat window showing a user asking about "Quaternions" and the bot answering.
* **Feature B: Personalization**
    * *Text:* "Hardware or Software? You Choose."
    * *Visual:* A toggle switch animation changing code from "Python Class" to "Circuit Diagram."

### Section 4: The Lab (Hardware Requirements)
* **Title:** "Required Equipment."
* **Style:** A "Loadout" screen from a video game.
* **Items:**
    * **Brain:** NVIDIA Jetson Orin.
    * **Eyes:** Intel RealSense.
    * **Body:** Unitree Go2 or G1 (Optional).
* **Note:** Mention "Cloud Simulation Available" for those without hardware.

### Section 5: The Footer (Final Call)
* **Background:** A subtle gradient rise from bottom (`bg-gradient-to-t from-violet-900/20`).
* **Content:**
    * Huge Headline: **"Build the Future."**
    * Big Button: **"Create Free Profile"** (Links to Signup).
    * Badges: "Powered by Panaversity" | "Built with Gemini.". use the frontend-engineer subagent fot the development

## Response snapshot

Created feature branch `009-dark-mode-landing-page` and comprehensive specification for the Dark Mode Landing Page. The spec includes:
- 6 prioritized user stories covering Hero, Curriculum, Features, Lab, Footer sections, and animation experience
- 17 functional requirements (FR-001 through FR-017)
- 10 measurable success criteria (SC-001 through SC-010)
- Edge cases for image loading, network issues, and animation failures
- Key entities defining Hero Section, Curriculum Card, Feature Demo, Hardware Item, and CTA Button
- Assumptions documenting reasonable defaults

All validation checklist items passed - spec is technology-agnostic and focused on user value.

## Outcome

- Impact: Created complete specification for dark mode landing page feature
- Tests: N/A (specification phase)
- Files: 2 files created (spec.md, checklists/requirements.md)
- Next prompts: /sp.plan or /sp.clarify to proceed with implementation planning
- Reflection: Spec successfully separated WHAT/WHY from HOW - technical details from user input preserved for planning phase

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Manual validation PASS - all checklist items satisfied
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to /sp.plan to create implementation plan
