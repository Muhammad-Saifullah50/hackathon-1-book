# Implementation Plan: Dark Mode Landing Page

**Branch**: `009-dark-mode-landing-page` | **Date**: 2025-12-12 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/009-dark-mode-landing-page/spec.md`

## Summary

Build a "NASA meets Cyberpunk" themed landing page for the Physical AI book with five main sections: Hero ("Wake Up the Metal"), Curriculum (4 glassmorphism cards), Features (AI Tutor + Personalization demos), Lab (hardware loadout), and Footer (conversion CTA). Implementation uses React/Docusaurus with Tailwind CSS and Framer Motion for scroll animations.

## Technical Context

**Language/Version**: TypeScript 5.6, React 19.0, Node.js 20+
**Primary Dependencies**:
- Docusaurus 3.9.2 (existing)
- Tailwind CSS 3.4.18 (existing)
- Framer Motion (to be installed)
- Lucide React 0.556.0 (existing)
- clsx/tailwind-merge (existing)

**Storage**: N/A (static landing page)
**Testing**: Jest 30.2, Playwright 1.57.0 (existing)
**Target Platform**: Web (modern browsers, responsive 320px-2560px)
**Project Type**: Web (Docusaurus frontend)
**Performance Goals**: Page fully renders < 3 seconds, animations complete < 500ms
**Constraints**: Accessibility score 90+, functional with JS disabled, Docusaurus theme compatibility
**Scale/Scope**: Single landing page, 5 sections, ~10 components

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| **Visual & UI Standards** | ✅ PASS | Clean, modern, futuristic design aligns with spec |
| **Aesthetic Goal** | ✅ PASS | "NASA meets Cyberpunk" fits high-tech academic standard |
| **Design Principles** | ✅ PASS | Whitespace, typography, cohesive color palette planned |
| **User Experience** | ✅ PASS | CTAs are intuitive, animations enhance not obstruct |
| **Test-Driven Development** | ✅ PASS | Will write component tests first |
| **Code Integrity** | ✅ PASS | Modular component architecture planned |
| **Safety (Sim-to-Real)** | N/A | Landing page only, no robotics code |

**Gate Result**: PASS - Proceed to Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/009-dark-mode-landing-page/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output (N/A - no API)
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
website/
├── src/
│   ├── components/
│   │   └── landing/           # NEW: Landing page components
│   │       ├── HeroSection.tsx
│   │       ├── CurriculumSection.tsx
│   │       ├── CurriculumCard.tsx
│   │       ├── FeaturesSection.tsx
│   │       ├── FeatureDemo.tsx
│   │       ├── ChatbotDemo.tsx
│   │       ├── PersonalizationDemo.tsx
│   │       ├── LabSection.tsx
│   │       ├── HardwareItem.tsx
│   │       ├── FooterCTA.tsx
│   │       ├── GlowButton.tsx
│   │       └── AnimatedSection.tsx
│   ├── pages/
│   │   ├── index.tsx          # MODIFY: Replace with new landing page
│   │   └── index.module.css   # MODIFY/DELETE: May replace with Tailwind
│   └── theme/                 # Existing theme overrides
├── static/
│   └── img/
│       └── landing/           # NEW: Landing page assets
│           ├── robot-hero.png (or .svg)
│           └── circuit-diagram.svg
└── tests/
    └── components/
        └── landing/           # NEW: Component tests
            ├── HeroSection.test.tsx
            ├── CurriculumCard.test.tsx
            └── GlowButton.test.tsx
```

**Structure Decision**: Extend existing Docusaurus website structure. All new components go in `src/components/landing/` to keep feature isolated and maintainable.

## Complexity Tracking

No violations - straightforward frontend implementation.

## Architecture Decisions

### AD-1: Framer Motion for Animations

**Decision**: Use Framer Motion library for scroll-triggered animations
**Rationale**:
- Industry standard for React animations
- Built-in `whileInView` API matches spec requirements exactly
- Supports reduced motion preferences via `useReducedMotion` hook
- Better performance than CSS-only solutions for complex scroll interactions

**Alternatives Rejected**:
- CSS-only animations: Insufficient control for scroll-triggered effects
- GSAP: Heavier bundle, overkill for this use case
- React Spring: Less intuitive API for scroll animations

### AD-2: Component-Based Section Architecture

**Decision**: Each landing page section is a self-contained component
**Rationale**:
- Aligns with spec's independent testability requirement
- Enables parallel development
- Easier maintenance and future modifications
- Clear separation of concerns

### AD-3: Tailwind CSS for Styling (No Custom CSS)

**Decision**: Use Tailwind utility classes exclusively, remove index.module.css
**Rationale**:
- Tailwind already configured in project
- Matches spec's specific class requirements (bg-slate-950, etc.)
- Faster development with utility-first approach
- Consistent with existing components in codebase

### AD-4: Static Demo Components (No Backend)

**Decision**: Feature demos (Chatbot, Personalization) are animated mocks, not functional
**Rationale**:
- Landing page scope is presentation only
- Real chatbot already exists (`ChatWidget.tsx`)
- Reduces complexity and page load time
- Spec explicitly calls for "mock chat window" and "toggle animation"
