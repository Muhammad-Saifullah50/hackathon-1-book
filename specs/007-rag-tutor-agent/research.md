# Research & Validation: The RAG Tutor Agent

**Feature**: The RAG Tutor Agent
**Branch**: `007-rag-tutor-agent`
**Date**: 2025-12-08

## Resolved Unknowns

### 1. Library Research

-   **`openai-agents` (Python)**:
    -   **Context7 ID**: `/openai/openai-agents-python`
    -   **Agent Definition**: Use `Agent` class with `name` and `instructions`.
    -   **Tool Registration**: Use `@function_tool` decorator.
    -   **Usage**: `Runner.run(agent, input)` executes the agent.
    -   **Caveat**: Specific RAG patterns in the SDK documentation were not explicitly found, but the tool usage is clear. I will implement RAG as a tool (`query_knowledge_base`).

-   **`openai-chatkit-react` & `openai-chatkit`**:
    -   **Context7 ID**: `/openai/chatkit-js`
    -   **Usage**: `useChatKit` hook provides `control`.
    -   **Components**: `<ChatKit control={control} />` is the main component.
    -   **Session Management**: `useChatKit` takes an `api` object with `getClientSecret`. This connects the frontend to the backend session endpoint.

-   **`Qdrant` Python Client**:
    -   **Context7 ID**: `/qdrant/qdrant-client`
    -   **Integration**: Standard `QdrantClient`, `search`, `upsert`.
    -   **RAG**: Embedding generation (e.g., via OpenAI or local model) will be handled before insertion/querying.

### 2. Feature Research: "Ask about Selection"

-   **Trigger**: Browser's `document.getSelection()` API.
-   **Integration**: A floating React component (using `createPortal` or just absolute positioning) that listens for selection events.
-   **Action**: On click, it opens the `ChatKit` widget (programmatically if supported, or via state) and sends a message with the selected text as context (e.g., "Explain this text: [selection]").
-   **ChatKit Helper**: `useChatKit` returns `sendUserMessage`, which can be used to programmatically send the selection context.

## Remaining Unknowns / Caveats

-   **Agent Session Persistence**: How `openai-agents-sdk` maintains context across turns. `Runner.run` seems stateless per call in basic examples, but `session` management via `openai-chatkit` likely handles the thread ID. The exact wiring between `ChatKit` session and `Agents SDK` runner needs careful implementation (passing thread ID).
-   **RAG Tool Output**: The RAG tool (`query_knowledge_base`) returns text. The Agent then synthesizes it. This is a standard pattern.

## Implementation Strategy Update

-   **Backend**: FastAPI service.
    -   Endpoint: `/api/chatkit/session` (generates token).
    -   Endpoint: `/api/chatkit/chat` (or similar, connected via WebSocket/HTTP by ChatKit). *Correction*: ChatKit likely connects directly to OpenAI's Realtime/Response API or a proxy. Since we are using `openai-agents-sdk` on the backend, we need a way to bridge ChatKit (frontend) to our Python Agent (backend).
    -   *Refinement*: `openai-chatkit` typically connects to a backend that implements the ChatKit protocol or proxies to OpenAI. With `openai-agents-sdk`, we might need to wrap the Agent Runner in a WebSocket or HTTP endpoint that ChatKit talks to. Or, if ChatKit is purely for OpenAI's direct API, we might need to use `OpenAI Realtime API` patterns. Given the "Agent Runtime" is `openai-agents-sdk`, I will assume a custom backend endpoint is needed to run the agent.
    -   *Actually*, looking at `openai-chatkit-advanced-samples`, it pairs a FastAPI backend. I will follow that pattern: Frontend -> ChatKit -> Backend (FastAPI + Agents SDK).

-   **Frontend**: React + `openai-chatkit-react`.
    -   Custom "Explain This" button using `useEffect` on `selectionchange`.
