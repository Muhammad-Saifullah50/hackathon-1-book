# Feature Specification: Page Content Personalization

**Feature Branch**: `011-personalize-page`
**Created**: 2025-12-14
**Status**: Draft
**Input**: User description: "Add personalize page feature. When user clicks personalize page button, request goes to backend with page content. Content goes to personalization agent built using OpenAI Agents. Agent builds dynamic instructions based on user profile and personalizes page content accordingly."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Personalize Current Page Content (Priority: P1)

As a learner reading a documentation page about Physical AI and Humanoid Robotics, I want to click a "Personalize Page" button and receive content tailored to my learning profile (technical background, learning style, goals), so that I can understand complex concepts more effectively.

**Why this priority**: This is the core value proposition of the feature. Without this, there is no personalization. It directly enables personalized learning experiences, which is fundamental to the platform's mission of making robotics education accessible to all skill levels.

**Independent Test**: Can be fully tested by clicking the personalize button on any documentation page and verifying the returned content matches the user's profile preferences (e.g., beginner gets simplified explanations, advanced users get technical depth).

**Acceptance Scenarios**:

1. **Given** a logged-in user with a completed profile (tech_background: "beginner") viewing a documentation page, **When** they click the "Personalize Page" button, **Then** the page content is replaced with a simplified version that uses analogies and avoids jargon.

2. **Given** a logged-in user with a completed profile (tech_background: "advanced", learning_mode: "hands_on") viewing a documentation page, **When** they click the "Personalize Page" button, **Then** the page content includes more technical depth, code examples, and practical exercises.

3. **Given** a logged-in user with a completed profile (preferred_language: "es") viewing a documentation page, **When** they click the "Personalize Page" button, **Then** the personalized content is returned in Spanish while maintaining technical accuracy.

---

### User Story 2 - Visual Loading State During Personalization (Priority: P2)

As a learner who has clicked the personalize button, I want to see a clear loading indicator while the content is being personalized, so that I know the system is working and can wait appropriately.

**Why this priority**: Good UX is essential for user trust. Without feedback, users may click repeatedly or think the system is broken. This enhances the P1 experience but isn't strictly required for MVP functionality.

**Independent Test**: Can be tested by clicking personalize and verifying the loading state appears immediately and dismisses when content arrives.

**Acceptance Scenarios**:

1. **Given** a user clicks the "Personalize Page" button, **When** the request is being processed, **Then** a loading indicator appears (e.g., shimmer effect or spinner on the content area) within 200ms.

2. **Given** personalization is in progress, **When** the personalized content returns successfully, **Then** the loading state is replaced with the personalized content smoothly.

3. **Given** personalization is in progress, **When** an error occurs, **Then** the loading state is replaced with an error message and option to retry.

---

### User Story 3 - Restore Original Content (Priority: P2)

As a learner who has viewed personalized content, I want to be able to restore the original (non-personalized) content, so that I can compare versions or see the standard documentation.

**Why this priority**: Provides user control and transparency. Users may want to see what others see, compare explanations, or prefer the original for reference. Enhances trust in the personalization system.

**Independent Test**: Can be tested by personalizing content, then clicking restore, and verifying original content reappears.

**Acceptance Scenarios**:

1. **Given** a user is viewing personalized content, **When** they click "Show Original", **Then** the original documentation content is displayed immediately (from local cache, no network call).

2. **Given** a user is viewing original content after having personalized it once, **When** they click "Show Personalized", **Then** the previously personalized version is shown (cached, no new request needed).

3. **Given** a user has previously personalized a page and navigates away then returns, **When** they land on that page, **Then** they see a "View Personalized" option that instantly loads the cached personalized content from localStorage (no API call).

---

### User Story 5 - Cross-Device Personalization Awareness (Priority: P2)

As a learner who previously personalized pages on another device (e.g., laptop), I want to be informed when visiting those pages on a new device (e.g., phone) and be able to re-personalize them for free, so that I don't lose my personalization work or waste my daily quota.

**Why this priority**: Cross-device support is essential for modern users who switch between devices. Without this, users would be penalized (lose quota) for using multiple devices, creating frustration.

**Independent Test**: Can be tested by personalizing a page on Device A, then logging in on Device B and verifying the "Re-personalize (Free)" option appears.

**Acceptance Scenarios**:

1. **Given** a user personalized a page on Device A (laptop), **When** they visit the same page on Device B (phone) where localStorage is empty, **Then** they see "Re-personalize (Free)" option with message "You personalized this before on another device".

2. **Given** a user clicks "Re-personalize (Free)" on a new device, **When** the personalization completes, **Then** their daily quota is NOT decremented (free re-personalization).

3. **Given** a user's profile has changed since the original personalization, **When** they visit a previously personalized page on any device, **Then** they see "Re-personalize with new profile (Free)" option.

---

### User Story 6 - Personalization for Unauthenticated Users (Priority: P3)

As a visitor who hasn't logged in or completed their profile, I want to understand why personalization isn't available and be guided toward creating an account, so that I can access this feature.

**Why this priority**: Conversion feature - helps turn visitors into registered users. Not required for core functionality but supports platform growth and user engagement.

**Independent Test**: Can be tested by viewing a page as a non-logged-in user and verifying the appropriate prompt appears.

**Acceptance Scenarios**:

1. **Given** a user is not logged in, **When** they see the personalization bar, **Then** they see a message like "Log in to personalize content to your learning style" with a link to the login page.

2. **Given** a user is logged in but has not completed their profile, **When** they click "Personalize Page", **Then** they see a prompt to complete their profile first with a link to the profile wizard.

---

### Edge Cases

- What happens when the personalization service is unavailable? Show an error message with retry option, keep original content visible.
- What happens when the page content is empty or very short? Personalize what exists, or return original if content is too minimal (< 100 characters).
- What happens when the user's profile is incomplete? Use available profile fields; fall back to "general audience" tone for missing fields.
- What happens when the personalization takes longer than expected (> 10 seconds)? Show timeout message with option to retry or view original.
- What happens when the user navigates away during personalization? Cancel the in-flight request, clean up loading state.
- What happens with special content like code blocks, images, or diagrams? Preserve code blocks and images unchanged; only personalize explanatory text.
- What happens when localStorage is full? Evict oldest personalized pages (LRU) to make room for new ones.
- What happens when server history API is unavailable? Fall back to localStorage-only mode; cross-device features gracefully degrade.
- What happens when original page content has changed since personalization? Detect via originalContentHash mismatch; offer "Re-personalize (content updated)" option.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST send the current page's markdown content to the backend when the user clicks "Personalize Page"
- **FR-002**: System MUST include the user's profile data (tech_background, learning_mode, learning_speed, primary_goal, education_level, preferred_language, focus_area) in the personalization request
- **FR-003**: Backend MUST use an AI agent (OpenAI Agents) to personalize content based on user profile
- **FR-004**: Agent MUST build dynamic instructions at runtime based on the specific user's profile attributes
- **FR-005**: System MUST return personalized content in the same markdown format as the original
- **FR-006**: System MUST preserve code blocks, images, and other non-text elements unchanged during personalization
- **FR-007**: System MUST display a loading state while personalization is in progress
- **FR-008**: System MUST handle errors gracefully with user-friendly messages and retry options
- **FR-009**: System MUST allow users to toggle between original and personalized content
- **FR-010**: System MUST cache personalized content in localStorage (persisted until user profile changes; cache invalidated on profile update)
- **FR-011**: System MUST require authentication for personalization requests
- **FR-012**: System MUST validate that the user has a profile before personalizing (prompt to complete profile if missing)
- **FR-013**: System MUST enforce a per-user daily limit of 5 personalizations per day (quota resets at midnight UTC)
- **FR-014**: System MUST display remaining personalization quota to users and show appropriate messaging when limit is reached
- **FR-015**: System MUST detect previously personalized pages on page load and show "View Personalized" option for instant retrieval from localStorage

#### Hybrid Caching (Cross-Device Support)

- **FR-016**: System MUST store personalization metadata (user_id, page_url, profile_hash, timestamp) server-side in database when a page is personalized
- **FR-017**: System MUST NOT store full personalized content server-side (only lightweight metadata ~100 bytes per record)
- **FR-018**: System MUST provide an API endpoint to fetch user's personalization history (list of page URLs they've personalized)
- **FR-019**: System MUST check server-side history on page load when localStorage cache is empty to detect cross-device personalization
- **FR-020**: System MUST offer "Re-personalize (Free)" option when user visits a previously personalized page on a new device
- **FR-021**: System MUST NOT decrement daily quota for re-personalization of previously personalized pages (free re-personalization)
- **FR-022**: System MUST invalidate both localStorage cache AND mark server-side metadata as stale when user profile changes
- **FR-023**: System MUST offer "Re-personalize with new profile (Free)" when profile_hash has changed since original personalization

#### Quota Tracking

- **FR-024**: System MUST track daily personalization usage per user in server-side database
- **FR-025**: System MUST return remaining quota in personalization API responses via response header or body

### Key Entities

- **PersonalizationRequest**: Contains page_content (markdown string), user_profile (profile attributes), page_url (for context/caching), is_free_repersonalization (boolean)
- **PersonalizationResponse**: Contains personalized_content (markdown string), original_preserved (boolean), processing_time_ms (for analytics), quota_remaining (integer)
- **UserProfile**: Existing entity with attributes: tech_background, learning_mode, learning_speed, primary_goal, education_level, preferred_language, focus_area, age_range, time_per_week

#### Server-Side Entities (Hybrid Caching)

- **PersonalizationHistory**: Lightweight metadata record containing user_id, page_url, profile_hash (SHA-256 of profile at personalization time), created_at timestamp. Unique constraint on (user_id, page_url). Does NOT store content.
- **PersonalizationQuota**: Daily usage tracking containing user_id, date (UTC), used_count (integer). Unique constraint on (user_id, date).

#### Client-Side Cache Schema (localStorage)

- **PersonalizedPageCache**: Stored per page with key format `personalized_${pageUrl}_${userId}`. Contains:
  - content: Full personalized markdown string
  - profileHash: SHA-256 hash of profile at personalization time
  - originalContentHash: Hash of original content (detect if source changed)
  - timestamp: ISO date of personalization
  - pageUrl: For validation

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can personalize a page within 15 seconds for typical page sizes (< 5000 words)
- **SC-002**: 90% of personalization requests complete successfully without errors
- **SC-003**: Personalized content maintains 100% of code blocks, images, and diagrams from the original
- **SC-004**: Users can toggle between original and personalized content within 100ms (cached)
- **SC-005**: System gracefully handles errors with clear messaging for 100% of failure cases
- **SC-006**: Personalized content accurately reflects at least 3 profile attributes (e.g., if beginner + visual learner + Spanish speaker, content should be simplified, include visual references, and be in Spanish)
- **SC-007**: Cross-device re-personalization completes without decrementing user's daily quota
- **SC-008**: Server-side metadata storage uses less than 200 bytes per personalization record (no content storage)

## Clarifications

### Session 2025-12-14

- Q: What API cost control strategy should be used? → A: Per-user daily limit of 5 personalizations/day
- Q: How long should personalized content cache persist? → A: Until profile changes (invalidate on profile update)
- Q: When should the daily personalization quota reset? → A: Midnight UTC (fixed global reset)
- Q: How should returning users see previously personalized pages? → A: Show "View Personalized" option on page load; instant load from localStorage (no API call)
- Q: How should previously personalized pages be preserved for cross-device access? → A: Hybrid approach - localStorage for instant access (primary), server-side metadata only for cross-device awareness (no content storage). Free re-personalization on new devices.

## Assumptions

- OpenAI Agents SDK (openai-agents) is already available in the backend environment
- The personalization agent will use GPT-4 or equivalent model for high-quality content transformation
- Users have a stable internet connection for personalization requests
- Page content is available as markdown that can be extracted from the current page
- The existing PersonalizationBar component will be enhanced (not replaced) to implement this feature
- Profile data is accessible via the existing useProfile hook on the frontend
- Backend authentication/authorization is already in place (JWT validation from Better Auth)
