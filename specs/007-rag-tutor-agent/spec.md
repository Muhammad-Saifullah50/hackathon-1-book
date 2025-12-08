# Feature Specification: The RAG Tutor Agent

**Feature Branch**: `007-rag-tutor-agent`  
**Created**: 2025-12-08  
**Status**: Draft  
**Input**: User description: "# Feature Spec: The RAG Tutor Agent (User Stories) ## 1. Role & Identity * **Name:** "The Physical AI Tutor." * **Persona:** A helpful, patient, and rigorous teaching assistant. It does not just give answers; it guides the student to the solution using Socratic questioning when appropriate. * **Knowledge Boundary:** It answers questions based **ONLY** on the textbook content. If a user asks about unrelated topics (e.g., "knitting"), it politely declines and pivots back to Robotics. ## 2. Core Capabilities (User Stories) * **Story: Global Q&A** * As a student, I want to ask broad questions like "How does ROS 2 differ from ROS 1?" and receive a clear answer that cites specific chapters in the book. * **Story: "Ask about Selection" (Contextual Help)** * As a reader, I want to highlight a specific confusing paragraph (e.g., about "Quaternion Math"), click a floating "Explain This" button, and get a simplified explanation of *just that text* without losing my place. * **Story: Code Explanation** * As a developer, I want to paste a code snippet from the book into the chat and ask "What does this specific line do?", receiving a line-by-line breakdown of the syntax and logic. * **Story: Sim-to-Real Advice** * As a hardware engineer, I want to ask "Will this code work on my Jetson?" and get a specific warning if the code is meant only for simulation. ## 3. UI/UX Specifications (The "Beautiful" Interface) * **The Widget:** * A floating "Chat Bubble" icon in the bottom-right corner that is unobtrusive but always accessible. * When clicked, it opens a clean, modern side panel (Sheet) or floating window (Popover) with a smooth slide-in animation. * **Interaction Design:** * **User Messages:** Distinctly colored bubbles (matching the primary theme). * **Agent Messages:** Clean, neutral background with full Markdown support (to render code blocks, lists, and bold text beautifully). * **Feedback:** * A subtle "Thinking..." animation (like a pulsing skeleton loader) to indicate the agent is processing, so the interface never feels frozen. * **Citations:** * Every answer must include clickable links to the source material (e.g., "Read more in Module 1, Lesson 2") so I can verify the information. ## 4. Agent Skills (Functional Tools) * **Skill: `simplify_concept`** * The agent can reword complex physics jargon into plain English when I ask it to "Explain like I'm 5." * **Skill: `quiz_me`** * The agent can generate a 3-question multiple-choice quiz based on the chapter I am currently reading to test my understanding. * **Skill: `debug_helper`** * The agent can analyze error logs I paste (e.g., "Node not found") and suggest fixes based on the book's troubleshooting guides."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Global Q&A with Citations (Priority: P1)

As a student, I want to ask broad questions about the course material (e.g., "How does ROS 2 differ from ROS 1?") and receive a clear, accurate answer that strictly adheres to the textbook content and provides clickable citations to specific chapters.

**Why this priority**: This is the primary function of the tutor: to make the knowledge base accessible and searchable via natural language.

**Independent Test**: Ask "What is the 'Three Brains' architecture?" and verify the answer cites Module 1, Chapter 1. Ask "How do I knit a sweater?" and verify the agent politely declines.

**Acceptance Scenarios**:

1. **Given** the chat widget is open, **When** I ask a question covered in the book, **Then** the agent provides an answer derived *only* from the content and includes a link to the relevant module/lesson.
2. **Given** the chat widget is open, **When** I ask an unrelated question (e.g., cooking, sports), **Then** the agent politely refuses and steers the conversation back to robotics.

### User Story 2 - "Ask about Selection" Contextual Help (Priority: P2)

As a reader, I want to highlight specific text on the page and click an "Explain This" button to get a simplified explanation of that specific context without navigating away.

**Why this priority**: This lowers the barrier to entry for complex topics by providing immediate, localized assistance.

**Independent Test**: Highlight a paragraph about "Quaternions" in Module 1, click "Explain This," and verify the chat opens with a simplified explanation of that specific paragraph.

**Acceptance Scenarios**:

1. **Given** I am reading a chapter, **When** I select text, **Then** a floating action button (e.g., "Explain This") appears.
2. **Given** I click the "Explain This" button, **When** the chat panel opens, **Then** it automatically sends a prompt contextually grounded in the selected text and returns a simplified explanation.

### User Story 3 - Code Explanation and Debugging (Priority: P2)

As a developer, I want to paste code snippets or error logs into the chat to get a line-by-line breakdown or troubleshooting advice based on the book's examples.

**Why this priority**: Robotics involves significant coding; understanding syntax and debugging errors are major hurdles for learners.

**Independent Test**: Paste a snippet of `rclpy` code and ask "What does this do?". Paste a "Node not found" error and ask for a fix.

**Acceptance Scenarios**:

1. **Given** a code snippet is pasted in the chat, **When** I ask for an explanation, **Then** the agent provides a breakdown of the syntax and logic.
2. **Given** an error log is pasted, **When** I ask for help, **Then** the agent suggests fixes based on known issues or troubleshooting guides within the textbook.

### User Story 4 - Sim-to-Real Advice (Priority: P3)

As a hardware engineer, I want to know if specific code or configurations are compatible with real hardware (e.g., Jetson Orin) versus simulation-only (Gazebo).

**Why this priority**: Bridging the gap between simulation and reality is a core learning objective of the course.

**Independent Test**: Ask "Can I run this Isaac Sim code on my Jetson?" and verify the agent warns about hardware constraints (e.g., lack of Ray Tracing support on Edge).

**Acceptance Scenarios**:

1. **Given** a question about hardware compatibility, **When** I ask about running simulation code on edge hardware, **Then** the agent provides specific warnings or confirmations based on the "Hardware Reality" sections of the book.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The agent MUST act as "The Physical AI Tutor" with a helpful, patient, and Socratic persona.
- **FR-002**: The agent MUST strictly limit its knowledge base to the provided textbook content (RAG - Retrieval Augmented Generation).
- **FR-003**: The agent MUST decline to answer questions outside the scope of the textbook (e.g., general knowledge unrelated to robotics).
- **FR-004**: The UI MUST feature a floating "Chat Bubble" in the bottom-right corner.
- **FR-005**: Clicking the bubble MUST open a side panel or popover with a slide-in animation.
- **FR-006**: The interface MUST support Markdown rendering (code blocks, lists, bold text) for agent messages.
- **FR-007**: The interface MUST show a "Thinking..." animation while the agent processes a request.
- **FR-008**: Every agent response containing factual claims MUST include clickable citations linking to the source module/lesson.
- **FR-009**: The system MUST support a "simplify_concept" capability (Explain like I'm 5).
- **FR-010**: The system MUST support a "quiz_me" capability (Generate 3 multiple-choice questions based on the current chapter).
- **FR-011**: The system MUST support a "debug_helper" capability (Analyze error logs against troubleshooting guides).
- **FR-012**: Implementation details for text selection trigger: Use browser's `document.getSelection()` API, a floating React component (using `createPortal` or absolute positioning), and `chatkit` context injection with `useChatKit`'s `sendUserMessage` to send the selected text.
- **FR-013**: RAG backend architecture: Use `Qdrant` for vector knowledge, a FastAPI backend using `openai-agents-sdk` to orchestrate RAG logic (query Qdrant -> prompt LLM).

### Key Entities

- **Agent**: The conversational interface.
- **Knowledge Base**: The indexed content of the textbook.
- **Citation**: A link to a specific source within the Knowledge Base.
- **Context**: The current chapter or selected text.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 95% of valid questions about textbook content receive an accurate answer with a correct citation.
- **SC-002**: 100% of off-topic questions are politely declined.
- **SC-003**: The "Explain This" feature works on any text selection within a documentation page.
- **SC-004**: UI interactions (open/close, message sending) occur with smooth animations ( < 300ms latency for UI response).