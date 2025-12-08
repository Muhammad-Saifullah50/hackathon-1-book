# Implementation Plan - The RAG Tutor Agent

**Feature**: The RAG Tutor Agent  
**Status**: Draft  
**Branch**: `007-rag-tutor-agent`

## Technical Context

-   **Frontend UI**: `openai-chatkit-react`. (Provides pre-built chat interface components matching Shadcn aesthetic).
-   **Client State**: `openai-chatkit`. (Manages WebSocket/HTTP connection, message history, optimistic updates).
-   **Agent Runtime**: `openai-agents`. (Backend framework for orchestration, tool calling, and context management).
-   **Vector Knowledge**: `Qdrant`. (Stores chunked textbook content for RAG).
-   **LLM**: Gemini/OpenAI (via `openai-agents-sdk`).
-   **Language**: TypeScript/React (Frontend), Python (Backend - FastAPI assumed based on `openai-agents-sdk` context).
-   **Data Flow**: User Input -> `ChatKit` -> Backend (`agents-sdk`) -> Qdrant (RAG) -> LLM -> Response Stream -> `ChatKit`.
-   **Strict Constraint**: API methods for `openai-agents-sdk` and `chatkit` MUST be retrieved from `context7`. No guessing.

## Constitution Check

-   **Principles**: The feature provides a helpful, patient teaching assistant, aligning with the project's educational goals.
-   **Architecture**: Separation of concerns: Frontend (UI/State) <-> Backend (Agent/RAG). Follows a Service-Oriented approach.
-   **Testing**: Functional tests for Q&A accuracy and UI interactions are implied.
-   **Security**: API keys for LLM and Qdrant must be managed securely (env vars).

## Phases

### Phase 0: Research (Mandatory `context7` Sourcing)

**Goal**: Retrieve authoritative documentation for the specific libraries `openai-agents-sdk` and `openai-chatkit`. Resolve implementation details for "Ask about Selection".

1.  **Library Research**:
    -   Task: Research `context7` for `openai-agents-sdk` (Python) usage, agent definition, and tool registration.
    -   Task: Research `context7` for `openai-chatkit` and `openai-chatkit-react` (usage, `ChatProvider`, `useChat`, component props).
    -   Task: Research `context7` for `Qdrant` Python client integration.
2.  **Feature Research**:
    -   Task: Investigate implementation for "Ask about Selection" (text selection trigger) – possibly using browser APIs and `chatkit` context injection.

### Phase 1: Design & Contracts

**Goal**: Define the Agent's structure, API, and RAG pipeline.

1.  **Data Model (`data-model.md`)**:
    -   Define the schema for RAG chunks (content, metadata: module/lesson).
    -   Define the Agent's tool definitions (`simplify_concept`, `quiz_me`, `debug_helper`).
2.  **Contracts**:
    -   Define the API contract between Frontend (`ChatKit`) and Backend (`Agents SDK`).
3.  **Quickstart (`quickstart.md`)**:
    -   Instructions for setting up Qdrant (local/cloud), indexing the book content, and running the Agent server.

### Phase 2: Implementation (Planned)

1.  **Backend Setup**:
    -   Initialize Qdrant collection and ingest book content (chunks with metadata).
    -   Set up `openai-agents-sdk` Agent with RAG logic (query Qdrant -> prompt LLM).
    -   Implement Agent Skills as tools.
2.  **Frontend Integration**:
    -   Install `openai-chatkit` packages.
    -   Implement `<ChatProvider>` and chat interface using `<Thread />`.
    -   Implement "Ask about Selection" logic (event listener -> open chat -> send context).
    -   Style components to match book theme.
3.  **Verification**:
    -   Test Global Q&A, Contextual Help, and Code Explanation flows.

## Gate Check

- [x] Technical Context clear?
- [x] Constitution satisfied?
- [ ] Unknowns resolved? (Pending Phase 0 Research)