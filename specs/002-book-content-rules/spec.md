# Feature Specification: Content Generation Rules (The "Beautiful Book" Standard)

**Feature Branch**: `002-book-content-rules`  
**Created**: 2025-12-08  
**Status**: Draft  
**Input**: User description: "$ARGUMENTS"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Standardized Chapter Creation (Priority: P1)

A content creator (or AI agent) writes a new book chapter that strictly adheres to the "Beautiful Book" structure and design standards, ensuring consistency and high pedagogical value.

**Why this priority**: This is the core purpose of the feature: to ensure all content meets the high standards defined for the book.

**Independent Test**: Create a sample `.mdx` chapter following all rules and verify it renders correctly with all required components (PersonalizationBar, Tabs, Alert, etc.).

**Acceptance Scenarios**:

1. **Given** a new blank chapter file, **When** the author writes content, **Then** the final file includes all 8 mandatory structural components in the correct order.
2. **Given** a data table is needed, **When** the author adds it, **Then** they use the Shadcn `<Table>` component instead of standard Markdown.
3. **Given** a simulation-based lesson, **When** the author describes physics, **Then** a Sim-to-Real warning `<Alert>` is included.
4. **Given** a code example, **When** it is presented, **Then** the hardware context (Workstation vs. Jetson) is explicitly stated.

### Edge Cases

- What happens when a chapter doesn't have a specific "Sim-to-Real" difference? (Assumption: The warning component might be optional if explicitly not applicable, but the rule says "Mandatory Component", so we assume it must be present or explicitly waived. For this spec, we will enforce it as mandatory per input).
- What happens if a diagram cannot be expressed in Mermaid.js? (Alternative: Embedded React component).

## Requirements *(mandatory)*

### Functional Requirements

#### 1. Chapter Structure (Standard Operating Procedure)
- **FR-001**: Every chapter file (`.mdx`) MUST include **Front Matter** with `Title`, `ID`, and `Experience Level` (Beginner/Advanced).
- **FR-002**: Every chapter MUST start with a **Hero Section** containing a brief, inspiring introduction.
- **FR-003**: Every chapter MUST include a **Learning Objectives** list.
- **FR-004**: Every chapter MUST include a `<PersonalizationBar />` (Shadcn component) immediately following objectives to tailor content for [Hardware Engineers] or [Software Engineers].
- **FR-005**: Every chapter MUST include a **Core Theory** section that uses analogies to explain concepts.
- **FR-006**: Every chapter MUST include an **Interactive Diagram** using Mermaid.js or an embedded React component.
- **FR-007**: Every **Code Lab** section MUST use Docusaurus `<Tabs>` for variations (e.g., "Python Agent" vs. "C++ Node") and include a "Copy" button.
- **FR-008**: Every chapter MUST include a **Sim-to-Real Warning** using a Shadcn `<Alert variant="destructive">` to highlight physics differences.

#### 2. UI/UX Enforcement
- **FR-009**: Content MUST NOT use standard Markdown tables; it MUST use Shadcn `<Table>` components.
- **FR-010**: Content MUST NOT use standard blockquotes; it MUST use Shadcn `<Card>` or `<Callout>` components.
- **FR-011**: Typography MUST use Tailwind classes for spacing and fonts (e.g., `className="prose prose-blue max-w-none"`).
- **FR-012**: Long text blocks MUST be broken up with relevant icons or `<Badge>` tags.

#### 3. Pedagogical Tone
- **FR-013**: The writing tone MUST adopt the "Professor" Persona: authoritative but encouraging.
- **FR-014**: Content MUST adhere to the "No Magic" Policy: explicitly explain *why* things work.
- **FR-015**: Content MUST explicitly state the **Hardware Truth**: whether code runs on the **Workstation** (Training) or the **Jetson** (Edge).

### Key Entities

- **Chapter**: Represents a single instructional unit (file).
- **Component**: Reusable UI elements (PersonalizationBar, Alert, Table, etc.).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of new chapters pass a static analysis or manual review for the presence of all 8 structural elements.
- **SC-002**: Zero instances of standard Markdown tables or blockquotes in the generated content.
- **SC-003**: All code blocks are accompanied by a hardware context indicator (Workstation vs. Jetson).
- **SC-004**: Readability scores (or reviewer feedback) confirm the "Professor" persona and use of analogies.