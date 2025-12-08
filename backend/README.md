# RAG Tutor Agent Backend

## Prerequisites

- Python 3.12+
- `uv` (for package management)
- Qdrant (running locally or cloud)
- OpenAI API Key

## Setup

1. Initialize environment:
   ```bash
   uv init
   uv venv
   source .venv/bin/activate
   uv pip install -r requirements.txt
   ```

2. Environment Variables:
   Ensure you have the following set (or in a `.env` file if using `python-dotenv`):
   ```bash
   export OPENAI_API_KEY="sk-..."
   export QDRANT_URL="http://localhost:6333"
   ```

## Running the Server

Run the development server from within the `backend/` directory:

```bash
# Correct command (do not use backend.main)
uv run uvicorn main:app --reload
```

The server will start at `http://localhost:8000`.

## Ingesting Data

To chunk and upload the textbook content to Qdrant:

```bash
# Run from the project root (one level up) because it reads 'website/docs'
cd ..
python3 scripts/ingest_book.py
```
