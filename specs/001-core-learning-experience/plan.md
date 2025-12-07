# Implementation Plan: Core Learning Experience

**Branch**: `001-core-learning-experience` | **Date**: 2025-12-07 | **Spec**: [specs/001-core-learning-experience/spec.md](spec.md)
**Input**: Feature specification from `/specs/001-core-learning-experience/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This feature establishes the core "Physical AI & Humanoid Robotics" textbook platform. It delivers the "Beautiful Book" UI using Docusaurus, Tailwind CSS, and Shadcn UI, implements the 13-week curriculum structure (ROS 2, Gazebo, Isaac Sim), and provides key learning tools like global search, interactive code blocks, and Urdu translation. The architecture focuses on a static site generator (SSG) approach for performance and ease of deployment, enriched with React interactive components.

## Technical Context

**Language/Version**: TypeScript 5.x, Node.js 18+ (LTS)
**Framework**: Docusaurus v3+ (React-based SSG)
**Styling**: Tailwind CSS (v3.4+) via PostCSS
**Component Library**: Shadcn UI (Radix UI + Tailwind)
**State Management**: React Context (for Personalization/Language state)
**Search**: Algolia DocSearch or Local Search (Docusaurus native) [NEEDS CLARIFICATION: Search Provider]
**Testing**: Jest + React Testing Library (Unit/Component), Playwright (E2E)
**Target Platform**: Web (Responsive: Desktop, Tablet, Mobile)
**Project Type**: Web application (Frontend-focused SSG)
**Performance Goals**: <1s Initial Content Paint, Instant Navigation (SPA)
**Constraints**: Accessibility (WCAG 2.1), Responsive Design, Offline capability (PWA potential) [NEEDS CLARIFICATION: PWA requirement?]

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

*   **Prime Directive (Textbook Platform)**: ✅ Aligned. Builds the platform for the textbook.
*   **Role & Identity (Expert/Engineer)**: ✅ Aligned. Tech stack (Docusaurus/React) represents modern engineering standards.
*   **Core Philosophy (Sim-to-Real)**: ✅ Aligned. Curriculum structure explicitly supports this flow.
*   **Pedagogical Standards (Clarity/Action)**: ✅ Aligned. Code blocks, clear typography, and navigation support this.
*   **Visual & UI Standards (Beautiful/Modern)**: ✅ Aligned. Shadcn + Tailwind is chosen specifically for this.
*   **Operational Guidelines (TDD)**: ✅ Aligned. Plan includes Jest/Playwright setup.
*   **Accuracy/Code Integrity**: ✅ Aligned. Code blocks will be syntax highlighted and verified.

## Project Structure

### Documentation (this feature)

```text
specs/001-core-learning-experience/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Docusaurus Project Structure
website/
├── docs/                   # Content (Markdown/MDX)
│   ├── module-1/           # "Foundations"
│   ├── module-2/           # "Digital Twin"
│   ├── ...
│   └── intro.md
├── src/
│   ├── components/         # Shadcn & Custom Components
│   │   ├── ui/             # Shadcn Primitives (Button, Card, etc.)
│   │   ├── CodeBlock/      # Custom Code Block Wrapper
│   │   ├── Sidebar/        # Custom Sidebar Enhancements
│   │   └── Features/       # Feature-specific components (Personalization, Translation)
│   ├── css/                # Global CSS (Tailwind directives)
│   ├── pages/              # Landing page, etc.
│   └── theme/              # Docusaurus Theme Swizzling
├── static/                 # Images, Robots.txt
├── docusaurus.config.ts    # Main Config
├── tailwind.config.js      # Tailwind Config
├── tsconfig.json           # TypeScript Config
└── package.json            # Dependencies

tests/
├── unit/                   # Jest tests for components
└── e2e/                    # Playwright tests for user journeys
```

**Structure Decision**: Standard Docusaurus structure (`website/`) configured for TypeScript, with a dedicated `tests/` directory at the root level for separation of concerns.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| (None)    |            |                                     |