# Tasks: The RAG Tutor Agent

**Feature**: The RAG Tutor Agent  
**Status**: Completed  
**Branch**: `007-rag-tutor-agent`

## Dependencies

- Phase 1 (Setup) -> Phase 2 (Research)
- Phase 2 (Research) -> Phase 3 (Backend), Phase 4 (Frontend)
- Backend and Frontend can be developed in parallel once research is complete.

## Implementation Strategy

1.  **MVP Scope**: Basic RAG functionality (Global Q&A) with minimal UI.
2.  **Incremental Delivery**: Add contextual help ("Ask about Selection") and advanced skills (Quiz, Debug) subsequently.
3.  **Context7 First**: Prioritize gathering all required `context7` information before implementation.

## Phase 1: Setup

**Goal**: Prepare project environment for RAG agent.

- [x] T001 Create directory `backend` for the Python agent service.
- [x] T002 Create directory `scripts` for data ingestion tools.
- [x] T003 Initialize Python environment (requirements.txt) with `openai-agents`, `fastapi`, `uvicorn`, `qdrant-client`.

## Phase 2: Research (`context7` Sourcing)

**Goal**: Resolve all `NEEDS CLARIFICATION` points and gather specific code/config snippets from `context7`.

- [x] T004 [P] Research `context7` for `openai-agents` (Python) usage, agent definition, and tool registration. Record exact snippets. (Resolved: Snippets in `research.md`)
- [x] T005 [P] Research `context7` for `openai-chatkit` and `openai-chatkit-react` (usage, `Chatkit`, `useChat`, component props). Record exact snippets. (Resolved: Snippets in `research.md`)
- [x] T006 [P] Research `context7` for `Qdrant` Python client integration (search, upsert). Record exact snippets. (Resolved: Snippets in `research.md`)
- [x] T007 [P] Research implementation for "Ask about Selection" (text selection trigger, context injection).

## Phase 3: User Story 1 (Global Q&A - Backend)

**Goal**: Implement the RAG backend and basic Q&A agent.
**Test**: Verify agent can answer questions based on ingested book content.

- [x] T008 [US1] Implement data ingestion script `scripts/ingest_book.py` to chunk `.mdx` files and upload to Qdrant.
- [x] T009 [US1] Define the RAG tool `query_knowledge_base` using `openai-agents` (`@function_tool`).
- [x] T010 [US1] Create the main Agent definition in `backend/agent.py` with instructions and the RAG tool.
- [x] T011 [US1] Implement the FastAPI server in `backend/main.py` to expose the agent (and session endpoints for ChatKit).

## Phase 4: User Story 1 (Global Q&A - Frontend)

**Goal**: Implement the Chat interface.
**Test**: Verify user can chat with the agent.

- [x] T012 [US1] Install `openai-chatkit` and `@openai/chatkit-react` in `website/`.
- [x] T013 [US1] Create `website/src/components/ChatWidget.tsx` using `<ChatProvider>` and `<Thread />` (or `<ChatKit />`).
- [x] T014 [US1] Implement session fetching logic (`getClientSecret`) in `ChatWidget.tsx` to connect to the backend.
- [x] T015 [US1] Add the `ChatWidget` to the Docusaurus layout (e.g., via `Root` or a wrapper).

## Phase 5: User Story 2 (Contextual Help - "Ask about Selection")

**Goal**: Implement "Explain This" feature for selected text.
**Test**: Verify selecting text triggers the chat with context.

- [x] T016 [US2] Create `website/src/components/SelectionPopup.tsx` to handle `selectionchange` events and show a button.
- [x] T017 [US2] Implement logic to open the ChatWidget and send a contextual message (e.g., "Explain this: [selection]") when the button is clicked.

## Phase 6: User Story 3 & 4 (Agent Skills)

**Goal**: Add specific skills for Code Explanation, Debugging, and Sim-to-Real advice.
**Test**: Verify agent handles code and hardware questions correctly.

- [x] T018 [US3] Refine Agent instructions in `backend/agent.py` to handle "Code Explanation" and "Sim-to-Real" queries specifically (persona tuning).
- [x] T019 [US3] (Optional) Implement `simplify_concept` and `debug_helper` as distinct tools if strictly necessary, or rely on prompt engineering within the main Agent.

## Phase 7: Polish

**Goal**: Final styling and cleanup.

- [x] T020 Style the ChatWidget to match the Docusaurus theme (Tailwind).
- [x] T021 Ensure citations (links to source modules) are correctly formatted by the Agent (prompt engineering).
- [x] T022 Verify mobile responsiveness of the chat widget.
