# Feature Specification: Urdu Translation Bar

**Feature Branch**: `012-urdu-translation`
**Created**: 2025-12-15
**Status**: Draft
**Input**: User description: "Add a translation bar below the personalization bar. It will have a button which says translate to Urdu. Clicking it will take the page's content to the backend. The backend API route will give the content to an OpenAI agent, named urdu_translation_agent. The content rendering pattern will be same as the personalization feature. The user should be able to toggle between the translated page and the original page."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Translate Page Content to Urdu (Priority: P1)

As a learner who prefers reading in Urdu, I want to click a "Translate to Urdu" button on any documentation page and receive the content translated into Urdu, so that I can understand complex Physical AI concepts in my native language.

**Why this priority**: This is the core value proposition of the feature. Without this, there is no translation capability. It directly enables Urdu-speaking learners to access educational content in their preferred language.

**Independent Test**: Can be fully tested by clicking the "Translate to Urdu" button on any documentation page and verifying the returned content is properly translated to Urdu while maintaining technical accuracy.

**Acceptance Scenarios**:

1. **Given** a user viewing a documentation page in English, **When** they click the "Translate to Urdu" button, **Then** the page content is replaced with an Urdu translation while preserving code blocks, images, and diagrams unchanged.

2. **Given** a user viewing a documentation page with technical terminology (e.g., "ROS 2", "URDF", "LiDAR"), **When** they click "Translate to Urdu", **Then** technical terms are appropriately handled (transliterated or kept in English where appropriate) with Urdu explanations.

3. **Given** a user clicks "Translate to Urdu" on a page with headings and structured content, **When** the translation completes, **Then** the document structure (headings, lists, paragraphs) is preserved in the translated output.

---

### User Story 2 - Visual Loading State During Translation (Priority: P2)

As a learner who has clicked the translate button, I want to see a clear loading indicator while the content is being translated, so that I know the system is working and can wait appropriately.

**Why this priority**: Good UX is essential for user trust. Translation may take several seconds, and without feedback, users may click repeatedly or think the system is broken.

**Independent Test**: Can be tested by clicking translate and verifying the loading state appears immediately and dismisses when translated content arrives.

**Acceptance Scenarios**:

1. **Given** a user clicks the "Translate to Urdu" button, **When** the request is being processed, **Then** a loading indicator appears (e.g., spinner or progress message "Translating...") within 200ms.

2. **Given** translation is in progress, **When** the translated content returns successfully, **Then** the loading state is replaced with the translated content smoothly.

3. **Given** translation is in progress, **When** an error occurs, **Then** the loading state is replaced with an error message and option to retry.

---

### User Story 3 - Toggle Between Original and Translated Content (Priority: P2)

As a learner who has viewed translated content, I want to be able to switch back to the original English content and toggle between versions, so that I can compare translations or reference the original documentation.

**Why this priority**: Provides user control and transparency. Users may want to see original technical terms, compare explanations, or verify translation accuracy.

**Independent Test**: Can be tested by translating content, then clicking "Show Original", and verifying original content reappears. Then clicking "Show Translated" to see cached translation.

**Acceptance Scenarios**:

1. **Given** a user is viewing translated Urdu content, **When** they click "Show Original", **Then** the original English content is displayed immediately (from local cache, no network call).

2. **Given** a user is viewing original content after having translated it once, **When** they click "Show Translated", **Then** the previously translated version is shown (cached, no new request needed).

3. **Given** a user has previously translated a page and navigates away then returns, **When** they land on that page, **Then** they see a "View Translated" option that instantly loads the cached translated content (no API call).

---

### User Story 4 - Translation for All Users (Priority: P2)

As any visitor to the platform (logged in or not), I want to be able to translate content to Urdu, so that language accessibility is available to everyone without requiring account creation.

**Why this priority**: Unlike personalization which requires profile data, translation is a standalone feature that benefits from maximum accessibility. This supports the platform's mission of making robotics education accessible.

**Independent Test**: Can be tested by visiting a page without logging in and verifying the translate button is available and functional.

**Acceptance Scenarios**:

1. **Given** a user is not logged in, **When** they view a documentation page, **Then** the "Translate to Urdu" button is visible and clickable.

2. **Given** a non-logged-in user clicks "Translate to Urdu", **When** the translation completes, **Then** they receive the translated content (cached in localStorage only, no server-side tracking).

3. **Given** a logged-in user clicks "Translate to Urdu", **When** the translation completes, **Then** the translation is tracked server-side for cross-device access.

---

### Edge Cases

- What happens when the translation service is unavailable? Show an error message with retry option, keep original content visible.
- What happens when the page content is empty or very short? Translate what exists, or return original if content is too minimal (< 50 characters).
- What happens when translation takes longer than expected (> 30 seconds)? Show timeout message with option to retry or view original.
- What happens when the user navigates away during translation? Cancel the in-flight request, clean up loading state.
- What happens with special content like code blocks, images, or diagrams? Preserve code blocks and images unchanged; only translate explanatory text and comments.
- What happens with inline code or technical commands? Preserve inline code unchanged; translate surrounding text.
- What happens when localStorage is full? Evict oldest translated pages (LRU) to make room for new ones.
- What happens when the original page content has changed since translation? Detect via originalContentHash mismatch; offer "Re-translate (content updated)" option.
- What happens with mixed RTL/LTR content? Ensure proper text direction for Urdu (RTL) while code blocks remain LTR.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST display a TranslationBar component below the PersonalizationBar on all documentation pages
- **FR-002**: TranslationBar MUST contain a "Translate to Urdu" button prominently displayed
- **FR-003**: System MUST send the current page's content to the backend when the user clicks "Translate to Urdu"
- **FR-004**: Backend MUST route translation requests to a dedicated API endpoint (e.g., `/api/translate/urdu`)
- **FR-005**: Backend MUST use an OpenAI Agent named `urdu_translation_agent` to translate content
- **FR-006**: Agent MUST preserve code blocks, inline code, images, and other non-text elements unchanged during translation
- **FR-007**: Agent MUST appropriately handle technical terminology (keep or transliterate technical terms while providing Urdu context)
- **FR-008**: System MUST return translated content in the same markdown format as the original
- **FR-009**: System MUST display a loading state while translation is in progress
- **FR-010**: System MUST handle errors gracefully with user-friendly messages and retry options
- **FR-011**: System MUST allow users to toggle between original and translated content
- **FR-012**: System MUST cache translated content in localStorage for instant retrieval
- **FR-013**: System MUST use the same content rendering pattern as the personalization feature (dual-container approach)
- **FR-014**: TranslationBar MUST be visually distinct from PersonalizationBar but follow the same design system
- **FR-015**: System MUST work for both authenticated and unauthenticated users
- **FR-016**: System MUST set proper text direction (RTL) for Urdu content display

#### Caching Requirements

- **FR-017**: System MUST cache translated content in localStorage with key format `translated_urdu_${pageUrl}`
- **FR-018**: Cache MUST include originalContentHash to detect when source content changes
- **FR-019**: Cache MUST include timestamp and expire after 30 days (re-translation required on next request)
- **FR-020**: For logged-in users, system MUST store translation metadata server-side for cross-device awareness

#### Quota Requirements

- **FR-021**: System MUST enforce a per-user daily limit of 5 translations per day (quota resets at midnight UTC)
- **FR-022**: For anonymous users, system MUST enforce quota based on IP address (5 translations/day per IP)
- **FR-023**: System MUST display remaining translation quota to users
- **FR-024**: System MUST show appropriate messaging when daily limit is reached
- **FR-025**: System MUST track daily translation usage per user/IP in server-side database

### Key Entities

- **TranslationRequest**: Contains page_content (markdown string), page_url (for context/caching), source_language ("en"), target_language ("ur")
- **TranslationResponse**: Contains translated_content (markdown string), processing_time_ms (for analytics), source_language, target_language
- **TranslatedPageCache** (localStorage): Contains content (translated markdown), originalContentHash, timestamp, pageUrl

#### Server-Side Entities (for logged-in users)

- **TranslationHistory**: Lightweight metadata record containing user_id (nullable for anonymous), page_url, target_language, created_at timestamp. Unique constraint on (user_id, page_url, target_language).
- **TranslationQuota**: Daily usage tracking containing user_id (or ip_address for anonymous), date (UTC), used_count (integer). Unique constraint on (user_id/ip_address, date).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can translate a typical documentation page (< 5000 words) within 30 seconds
- **SC-002**: 90% of translation requests complete successfully without errors
- **SC-003**: Translated content maintains 100% of code blocks, images, and diagrams from the original
- **SC-004**: Users can toggle between original and translated content within 100ms (cached)
- **SC-005**: System gracefully handles errors with clear messaging for 100% of failure cases
- **SC-006**: Translated content displays correctly with proper RTL text direction for Urdu
- **SC-007**: Technical terms are handled consistently (either transliterated or kept with Urdu explanation)
- **SC-008**: Translation feature is accessible to 100% of users (no authentication required)

## Clarifications

### Session 2025-12-15

- Q: What API cost control strategy should be used for translations? → A: Same quota model as personalization (5 translations/day per user or IP)
- Q: What cache expiration policy should be used for translated content? → A: 30-day expiration (translations older than 30 days are re-translated on next request)

## Assumptions

- OpenAI Agents SDK (openai-agents) is already available in the backend environment (from personalization feature)
- The translation agent will use GPT-4 or equivalent model for high-quality translations
- Users have a stable internet connection for translation requests
- Page content is available as markdown that can be extracted from the current page (same as personalization)
- The existing PersonalizationBar component's rendering pattern can be reused for translation
- The translation bar appears below the personalization bar in the DOM hierarchy
- Rate limiting follows the same model as personalization (5/day per user or IP)
- The feature focuses on Urdu only initially; other languages may be added in future iterations
