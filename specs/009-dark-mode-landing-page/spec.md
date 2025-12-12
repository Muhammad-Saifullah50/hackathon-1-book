# Feature Specification: Dark Mode Landing Page

**Feature Branch**: `009-dark-mode-landing-page`
**Created**: 2025-12-12
**Status**: Draft
**Input**: User description: "Build the landing page for Physical AI book with NASA meets Cyberpunk theme"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - First Impression & Navigation (Priority: P1)

A visitor lands on the Physical AI book website and immediately understands the book's value proposition through the Hero section. They see the compelling headline "Wake Up the Metal" with gradient styling, a clear subheadline explaining the book covers Physical AI, ROS 2, and Humanoid Mechanics, and prominent call-to-action buttons.

**Why this priority**: The Hero section is the first thing users see and determines whether they continue exploring or leave. It establishes brand identity and communicates core value in seconds.

**Independent Test**: Can be fully tested by loading the homepage and verifying the Hero section displays correctly with headline, subheadline, robot visual, and both CTA buttons functional.

**Acceptance Scenarios**:

1. **Given** a user navigates to the homepage, **When** the page loads, **Then** they see the Hero section with "Wake Up the Metal" headline where "Metal" has a cyan-to-violet gradient effect
2. **Given** a user views the Hero section, **When** they look for actions, **Then** they see a prominent "Start Reading" button (solid cyan) and "Watch Demo" button (outlined white)
3. **Given** a user clicks "Start Reading", **When** the button is activated, **Then** they are navigated to the book's first chapter or table of contents
4. **Given** a user views the page on any device, **When** the Hero section renders, **Then** the layout adapts responsively (centered on mobile, split on desktop)

---

### User Story 2 - Understanding the Curriculum (Priority: P1)

A prospective learner wants to understand what the book covers and how the content is organized. They scroll to the Curriculum section and see four glass-styled cards representing the learning journey, each with an icon, title, and week range.

**Why this priority**: Understanding the curriculum scope is essential for users to evaluate if this book meets their learning goals. It drives conversion decisions.

**Independent Test**: Can be fully tested by scrolling to the Curriculum section and verifying all four cards display with correct content, glassmorphism styling, and icons.

**Acceptance Scenarios**:

1. **Given** a user scrolls past the Hero, **When** the Curriculum section enters view, **Then** it fades in smoothly with the title "The Journey to Embodied Intelligence"
2. **Given** a user views the Curriculum section, **When** all cards load, **Then** they see four glassmorphism-styled cards: "The Nervous System" (ROS 2), "The Digital Twin" (Simulation), "The Brain" (NVIDIA Isaac), "The Body" (VLA & Humanoid Control)
3. **Given** a user views any curriculum card, **When** examining the card, **Then** they see an appropriate icon, a title, and the week range for that topic
4. **Given** a user views the section on mobile, **When** the cards render, **Then** they stack vertically in a single column

---

### User Story 3 - Exploring Interactive Features (Priority: P2)

A potential reader wants to understand the book's unique interactive features. They view the Features section showcasing the AI Tutor (RAG Chatbot) and Personalization options with animated visual demonstrations.

**Why this priority**: Interactive features differentiate this book from competitors and demonstrate added value, but users first need to understand the core curriculum.

**Independent Test**: Can be fully tested by viewing the Features section and verifying the zig-zag layout displays both features with animations.

**Acceptance Scenarios**:

1. **Given** a user scrolls to the Features section, **When** the AI Tutor feature enters view, **Then** they see text "Never Get Stuck. Ask the RAG Chatbot" with an animated mock chat window showing a conversation about "Quaternions"
2. **Given** a user continues scrolling, **When** the Personalization feature enters view, **Then** they see text "Hardware or Software? You Choose" with a toggle animation showing content switching between Python code and circuit diagram
3. **Given** a user views Features on desktop, **When** examining the layout, **Then** Feature A shows text left/visual right, and Feature B shows text right/visual left (zig-zag pattern)

---

### User Story 4 - Evaluating Hardware Requirements (Priority: P2)

A reader evaluating the book wants to know what equipment they need. They view the Lab section styled like a video game "loadout" screen, showing required hardware with a note about cloud alternatives.

**Why this priority**: Hardware requirements can be a barrier to entry; providing clear information with cloud alternatives removes objections.

**Independent Test**: Can be fully tested by viewing the Lab section and verifying all hardware items display with cloud simulation alternative mentioned.

**Acceptance Scenarios**:

1. **Given** a user scrolls to the Lab section, **When** the section loads, **Then** they see "Required Equipment" title with a game-inspired visual style
2. **Given** a user examines the Lab section, **When** viewing equipment list, **Then** they see three items: "Brain" (NVIDIA Jetson Orin), "Eyes" (Intel RealSense), "Body" (Unitree Go2 or G1 - Optional)
3. **Given** a user is concerned about hardware costs, **When** viewing the section, **Then** they see a note stating "Cloud Simulation Available" for those without physical hardware

---

### User Story 5 - Converting to Signup (Priority: P1)

A convinced visitor wants to create an account to access the book. They reach the Footer section with a compelling final call-to-action and create their free profile.

**Why this priority**: Conversion is the ultimate goal. A strong footer CTA captures users who have scrolled through all content and are ready to commit.

**Independent Test**: Can be fully tested by scrolling to the Footer and verifying the CTA button links to signup page.

**Acceptance Scenarios**:

1. **Given** a user scrolls to the page footer, **When** the Footer renders, **Then** they see a large headline "Build the Future" with a gradient background rising from the bottom
2. **Given** a user wants to sign up, **When** they click "Create Free Profile" button, **Then** they are navigated to the signup page
3. **Given** a user views the footer, **When** examining the bottom content, **Then** they see badges stating "Powered by Panaversity" and "Built with Gemini"

---

### User Story 6 - Smooth Scrolling Experience (Priority: P3)

A user scrolling through the page experiences smooth, animated transitions as each section comes into view, creating an engaging "NASA meets Cyberpunk" atmosphere.

**Why this priority**: Animations enhance user experience but are secondary to core content delivery. The page should work without animations.

**Independent Test**: Can be fully tested by scrolling through the page and observing fade-in-up animations on each section.

**Acceptance Scenarios**:

1. **Given** a user scrolls down the page, **When** any section enters the viewport, **Then** the section animates with a fade-in-up effect (opacity 0 to 1, y offset 20 to 0)
2. **Given** a user has reduced motion preferences enabled, **When** viewing the page, **Then** animations are reduced or disabled to respect accessibility settings

---

### Edge Cases

- What happens when the robot image fails to load? Display a fallback placeholder maintaining layout integrity.
- How does the page handle slow network connections? Progressive loading with skeleton states for heavy content.
- What if a user navigates directly to a non-existent CTA target? Buttons should link to valid pages or show graceful 404 handling.
- How does the page appear before animations complete? Content should be visible (not hidden) even if animations fail.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Page MUST display a Hero section with gradient-styled headline "Wake Up the Metal" where "Metal" shows cyan-to-violet gradient
- **FR-002**: Page MUST include two CTA buttons in Hero: "Start Reading" (solid cyan background) and "Watch Demo" (white border, transparent background)
- **FR-003**: Page MUST display a high-quality robot visual (Unitree G1 or wireframe robot hand) in the Hero section
- **FR-004**: Page MUST render four curriculum cards with glassmorphism styling (semi-transparent background, backdrop blur, thin white border)
- **FR-005**: Each curriculum card MUST display an appropriate icon, title, and week range
- **FR-006**: Page MUST display Features section with zig-zag layout (alternating text/visual placement)
- **FR-007**: AI Tutor feature MUST show an animated mock chat window demonstrating a "Quaternions" conversation
- **FR-008**: Personalization feature MUST show a toggle animation between Python code and circuit diagram views
- **FR-009**: Page MUST display Lab section with hardware requirements styled as a game loadout screen
- **FR-010**: Lab section MUST include "Cloud Simulation Available" messaging
- **FR-011**: Footer MUST display "Build the Future" headline with gradient background
- **FR-012**: Footer MUST include "Create Free Profile" button linking to signup page
- **FR-013**: Footer MUST display "Powered by Panaversity" and "Built with Gemini" badges
- **FR-014**: All sections MUST animate with fade-in-up effect when scrolling into view
- **FR-015**: Page MUST use deep void background color throughout
- **FR-016**: All interactive buttons MUST have subtle glow effects
- **FR-017**: Page MUST be fully responsive across mobile, tablet, and desktop viewports

### Key Entities

- **Hero Section**: Primary landing content with headline, subheadline, robot visual, and CTA buttons
- **Curriculum Card**: Glass-styled card containing icon, title, subtitle, and week range representing a learning module
- **Feature Demo**: Interactive demonstration component with text description and animated visual
- **Hardware Item**: Equipment specification showing name, description, and optional status
- **CTA Button**: Action trigger with primary (solid) and secondary (outlined) variants

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Page fully renders within 3 seconds on standard broadband connection
- **SC-002**: All five main sections (Hero, Curriculum, Features, Lab, Footer) are visible and functional
- **SC-003**: Users can navigate from Hero CTA to book content in one click
- **SC-004**: 100% of curriculum cards display with correct glassmorphism styling and icons
- **SC-005**: Both interactive feature demos animate correctly when scrolled into view
- **SC-006**: Page maintains visual integrity across screen sizes from 320px to 2560px width
- **SC-007**: All animations complete within 500ms of section entering viewport
- **SC-008**: Footer signup button successfully navigates to signup page
- **SC-009**: Page achieves accessibility score of 90+ on automated accessibility testing
- **SC-010**: Page functions correctly with JavaScript disabled (content visible, links work)

## Assumptions

- Robot imagery (Unitree G1 or wireframe) will be provided or a placeholder will be used initially
- Signup page exists at `/signup` route
- Book content exists at `/docs` or similar route for "Start Reading" navigation
- "Watch Demo" may link to an external video or placeholder initially
- Icons for curriculum cards will use standard icon library (Lucide or similar available in the project)
- The existing Docusaurus website has Tailwind CSS configured or will be configured as part of implementation
- Framer Motion library will be installed for animations
