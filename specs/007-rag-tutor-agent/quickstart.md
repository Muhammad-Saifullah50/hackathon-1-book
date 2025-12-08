# Quickstart: RAG Tutor Agent Setup

## Prerequisites

-   Python 3.10+
-   Node.js 18+
-   Qdrant (Docker or Cloud)
-   OpenAI API Key

## Backend Setup (FastAPI + Agents SDK)

1.  **Install Dependencies**:
    ```bash
    pip install openai-agents-sdk fastapi uvicorn qdrant-client openai
    ```

2.  **Configure Environment**:
    ```bash
    export OPENAI_API_KEY="sk-..."
    export QDRANT_URL="http://localhost:6333"
    ```

3.  **Ingest Knowledge Base**:
    *   Run the ingestion script (to be created) to parse `.mdx` files and upload to Qdrant.
    ```bash
    python scripts/ingest_book.py
    ```

4.  **Start Backend**:
    ```bash
    uvicorn backend.main:app --reload
    ```

## Frontend Setup (React + ChatKit)

1.  **Install Dependencies**:
    ```bash
    cd website
    npm install @openai/chatkit-react
    ```

2.  **Run Docusaurus**:
    ```bash
    npm start
    ```

## Verification

1.  Open the website.
2.  Click the chat bubble.
3.  Ask: "What is the Three Brains architecture?"
4.  Verify the agent responds with a citation.
