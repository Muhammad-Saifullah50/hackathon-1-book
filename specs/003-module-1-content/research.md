# Research & Validation: Module 1 Content

**Feature**: Module 1 Content
**Branch**: `003-module-1-content`
**Date**: 2025-12-08

## Component Audit

**Goal**: Verify availability of required UI components.

- **Status**:
    - `<PersonalizationBar />`: **MISSING**. Needs implementation.
    - `<Card />`: **MISSING** (in `src/components/ui`). Needs implementation.
    - `<Alert />`: **MISSING** (in `src/components/ui`). Needs implementation.
    - `<Table />`: **MISSING** (in `src/components/ui`). Needs implementation.
    - `<Badge />`: **MISSING** (in `src/components/ui`). Needs implementation.
    - `<Tabs />`: **AVAILABLE** (Docusaurus built-in).

**Decision**: Implement the missing components in `website/src/components/ui` or `website/src/components/shared` as part of the implementation phase. They should use Tailwind CSS for styling to match the "Shadcn" aesthetic.

## Mermaid.js Support

**Goal**: Verify Mermaid diagram support.

- **Status**: Not currently configured in `docusaurus.config.ts`.
- **Action**: Need to install `@docusaurus/theme-mermaid` and add it to the configuration.

## Content Structure

- **Location**: `website/docs/module-01/`
- **Files**:
    - `01-embodied-intelligence.mdx`
    - `02-ros2-fundamentals.mdx`
    - `03-urdf-and-tf.mdx`
    - `04-capstone-project.mdx`
- **Front Matter**:
    - `title`: string
    - `sidebar_label`: string (optional, usually same as title or shorter)
    - `description`: string (for SEO/metadata)
    - `custom_edit_url`: null (to disable edit link if needed)
    - `experience_level`: 'Beginner' | 'Advanced' (Custom field)

## Dependencies

- **Install**:
    - `@docusaurus/theme-mermaid`
    - `lucide-react` (already present)
    - `clsx`, `tailwind-merge` (already present)
