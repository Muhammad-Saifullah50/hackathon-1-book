# Feature Specification: Authentication & Detailed Profiling

**Feature Branch**: `008-learner-auth-profile`
**Created**: 2025-12-09
**Status**: Draft
**Input**: User description provided via CLI.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - The "Learner Profile" Wizard (Priority: P1)

As a new user, I want a multi-step signup process that feels like setting up a game character, not filing taxes, so that I can personalize my learning experience.

**Why this priority**: Essential for the core value proposition of "Learner DNA" and personalization.

**Independent Test**: Can be tested by going through the signup flow and verifying the profile data is saved in Supabase.

**Acceptance Scenarios**:

1. **Given** a new user on the signup page, **When** they start the wizard, **Then** they see Step 1 (Basics: Email, Password, Age Range).
2. **Given** a user completing Step 1, **When** they proceed, **Then** they see Step 2 (Background: Education Level, Tech Stack History).
3. **Given** a user completing Step 2, **When** they proceed, **Then** they see Step 3 (Strategy: Goals, Time per Week, Learning Speed).
4. **Given** a user on any step, **When** they choose "Skip for now", **Then** they are registered with a partial profile and notified that personalization is disabled.
5. **Given** a user completes the wizard, **When** they submit, **Then** a user record and profile are created in Supabase with all selected enums.

---

### User Story 2 - The "Personalize" Trigger (Priority: P2)

As a user reading Chapter 1, I want to see a persistent **Personalization Bar** at the top.

**Why this priority**: This is the UI entry point for the personalization features.

**Independent Test**: Can be tested by viewing Chapter 1 and verifying the bar exists and responds to interaction.

**Acceptance Scenarios**:

1. **Given** a logged-in user reading Chapter 1, **When** the page loads, **Then** a persistent "Personalization Bar" is visible at the top.
2. **Given** the Personalization Bar, **When** the user clicks "Personalize Page", **Then** the system triggers the personalization logic (logic implementation deferred, but event must fire).

### Edge Cases

- **User Abandons Wizard**: If a user closes the browser before the final submit, no account/profile is created (unless per-step saving is implemented, which is not specified; assumed atomic submit).
- **Missing Profile Data**: If a user who "Skipped" the wizard clicks "Personalize Page", the system should handle the missing data gracefully (e.g., prompt to complete profile or use defaults).
- **Invalid/Malicious Input**: API requests attempting to send invalid enum values (e.g., "Wizard" for "Learning Mode") must be rejected by the backend validation.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST use **better-auth** for authentication.
- **FR-002**: System MUST store user profile data in **Supabase** (PostgreSQL).
- **FR-003**: System MUST provide a multi-step wizard UI using **Shadcn UI** components.
- **FR-004**: System MUST validate all enum fields using **Zod** against the defined schema (Age Range, Education Level, etc.).
- **FR-005**: System MUST allow users to skip the detailed profiling steps ("Skip for now").
- **FR-006**: System MUST explicitly state that profile data is used *only* for tailoring the textbook experience (Privacy Notice).
- **FR-007**: System MUST support the following data fields for a user profile: `age_range`, `education_level`, `tech_background`, `primary_goal`, `learning_mode`, `learning_speed`, `time_per_week`, `preferred_language`.
- **FR-008**: System MUST default `preferred_language` to 'en' (English).

### Key Entities

- **User Profile**:
    - `age_range`: Enum ('under_18', '18_24', '25_34', '35_plus')
    - `education_level`: Enum ('high_school', 'undergrad', 'masters', 'phd', 'self_taught')
    - `tech_background`: Enum ('software_engineer', 'hardware_engineer', 'student', 'hobbyist')
    - `primary_goal`: Enum ('career_switch', 'academic', 'hobby_project', 'startup_founder')
    - `learning_mode`: Enum ('visual', 'textual', 'code_first')
    - `learning_speed`: Enum ('intensive', 'balanced', 'casual')
    - `time_per_week`: Integer
    - `preferred_language`: String (default 'en')

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can complete the full 3-step profile wizard in under 2 minutes.
- **SC-002**: 100% of created profiles have valid enum values enforced by the database and API.
- **SC-003**: The "Personalize Page" action is successfully logged/triggered for 100% of clicks.
- **SC-004**: Users skipping the wizard can still access the content (without personalization).