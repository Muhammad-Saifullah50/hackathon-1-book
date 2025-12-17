# Physical AI & Humanoid Robotics Backend

> **RAG-powered AI Tutor API with FastAPI, OpenAI Agents, and Neon PostgreSQL**

A production-ready async Python backend providing intelligent tutoring, user authentication, personalization, and chat persistence for the Physical AI & Humanoid Robotics learning platform.

---

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Technology Stack](#technology-stack)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Configuration](#configuration)
- [Development](#development)
- [API Documentation](#api-documentation)
- [RAG System](#rag-system)
- [Database](#database)
- [Testing](#testing)
- [Deployment](#deployment)
- [Troubleshooting](#troubleshooting)

---

## Overview

This FastAPI backend powers an interactive learning experience for graduate-level robotics education. It combines:

- **RAG-Powered Tutor Agent**: Uses OpenAI Agents framework with Qdrant vector database to answer questions about ROS 2, Isaac Sim, and Vision-Language-Action models
- **Persistent Chat History**: Stores conversation threads in Neon PostgreSQL for authenticated and anonymous users
- **User Authentication**: JWT-based auth integration with Better Auth
- **Personalization Engine**: AI agent that adapts content to learner profiles
- **Translation Service**: Real-time Urdu translation for accessibility
- **Async Architecture**: Built on FastAPI with asyncpg for high-performance database operations

### Key Features

- Real-time streaming responses via ChatKit Server-Sent Events (SSE)
- Automatic thread title generation using Gemini AI
- Context-aware retrieval from textbook content (4 modules, 13 weeks)
- Rate limiting and quota management for AI features
- CORS-enabled for cross-origin requests from Docusaurus frontend

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     FastAPI Application                      │
│                         (main.py)                            │
└───────────────────────┬─────────────────────────────────────┘
                        │
        ┌───────────────┼───────────────┐
        │               │               │
        ▼               ▼               ▼
┌──────────────┐ ┌──────────────┐ ┌──────────────┐
│ RAG Tutor    │ │ Profile API  │ │ Features API │
│ (/chatkit)   │ │ (/profile)   │ │ (/personalize│
│              │ │              │ │  /translate) │
└──────┬───────┘ └──────┬───────┘ └──────┬───────┘
       │                │                │
       │                │                │
       ▼                ▼                ▼
┌──────────────────────────────────────────────┐
│           Services & Business Logic          │
│  - ChatStore (NeonChatKitStore)             │
│  - ProfileService                            │
│  - PersonalizationService                    │
│  - TranslationService                        │
│  - JWT Validation (Better Auth)              │
└──────┬──────────────────────────┬────────────┘
       │                          │
       ▼                          ▼
┌─────────────────┐      ┌──────────────────┐
│ Neon PostgreSQL │      │ Qdrant Vector DB │
│ - chat_threads  │      │ - RAG embeddings │
│ - profiles      │      │ - textbook chunks│
│ - quotas        │      └──────────────────┘
└─────────────────┘
       │
       ▼
┌─────────────────┐
│ AI Agents       │
│ - RAG Tutor     │
│ - Personalizer  │
│ - Translator    │
│ - Title Gen     │
└─────────────────┘
```

### Request Flow (RAG Tutor Example)

1. **Client Request**: Frontend sends POST `/chatkit` with user message
2. **Authentication**: JWT token validated (optional for anonymous users)
3. **ChatKit Server**: Routes request to `RagTutorChatKitServer`
4. **Thread Management**: Loads thread metadata + history from Neon
5. **Agent Execution**:
   - Query converted to embedding via Gemini `text-embedding-004`
   - Qdrant searches top 3 relevant textbook chunks
   - RAG Tutor Agent synthesizes answer with citations
6. **Streaming Response**: SSE stream sent back to client
7. **Persistence**: Messages saved to `chat_thread_items` table

---

## Technology Stack

### Core Framework
| Technology | Version | Purpose |
|------------|---------|---------|
| **Python** | 3.12+ | Runtime environment |
| **FastAPI** | Latest | Async web framework |
| **uvicorn** | Latest | ASGI server |
| **Pydantic** | 2.12.5+ | Data validation & serialization |

### AI & Embeddings
| Technology | Purpose |
|------------|---------|
| **openai-agents** | Agent framework for RAG Tutor |
| **openai-chatkit** | ChatKit server protocol implementation |
| **google-genai** | Gemini API for embeddings & title generation |
| **qdrant-client** | Vector database client for semantic search |

### Database
| Technology | Version | Purpose |
|------------|---------|---------|
| **Neon PostgreSQL** | Serverless | Primary database |
| **asyncpg** | 0.31.0+ | Async PostgreSQL driver |
| **psycopg2-binary** | 2.9.9+ | Sync PostgreSQL driver (migrations) |

### Authentication & Security
| Technology | Version | Purpose |
|------------|---------|---------|
| **PyJWT** | 2.8.0+ | JWT validation for Better Auth |
| **httpx** | 0.27.0+ | Async HTTP client for JWKS |

### Development
| Technology | Purpose |
|------------|---------|
| **pytest** | Unit & integration testing |
| **pytest-asyncio** | Async test support |
| **python-dotenv** | Environment variable management |

---

## Project Structure

```
backend/
├── main.py                    # FastAPI app entry point
├── agent.py                   # RAG Tutor Agent definition
├── agent_tools.py             # Knowledge base query tool
├── chatkit_server.py          # ChatKit server implementation
├── pyproject.tomllll          # uv project dependencies
├── requirements.txt           # pip-compatible requirements
├── .env                       # Environment variables (DO NOT COMMIT)
├── .python-version            # Python 3.12
│
├── src/
│   ├── api/                   # API route handlers
│   │   ├── auth/
│   │   │   └── dependencies.py        # JWT validation & current user
│   │   ├── profile/
│   │   │   └── routes.py              # User profile CRUD
│   │   ├── personalization/
│   │   │   └── routes.py              # Personalization endpoint
│   │   └── translation/
│   │       └── routes.py              # Translation endpoint
│   │
│   ├── services/              # Business logic layer
│   │   ├── database.py                # Neon connection pool
│   │   ├── chat_store.py              # ChatKit Store (Neon)
│   │   ├── profile_service.py         # Profile CRUD operations
│   │   ├── personalization_service.py # Personalization logic
│   │   ├── translation_service.py     # Translation logic
│   │   └── jwt_service.py             # JWT validation service
│   │
│   ├── models/                # Pydantic data models
│   │   ├── auth.py                    # Auth request/response models
│   │   ├── profile.py                 # User profile schema
│   │   ├── chat.py                    # Chat message models
│   │   ├── personalization.py         # Personalization models
│   │   └── translation.py             # Translation models
│   │
│   └── ai_agents/             # AI agent implementations
│       ├── personalization_agent.py   # Content personalization
│       └── translation_agent.py       # Urdu translation
│
├── models/
│   └── gemini.py              # Gemini model configuration
│
├── scripts/
│   └── migrations/            # SQL migration scripts
│       ├── 003_create_personalization_tables.sql
│       ├── 004_create_translation_tables.sql
│       └── 005_create_chat_tables.sql
│
└── tests/
    ├── unit/                  # Unit tests (services, models)
    ├── integration/           # Integration tests (API routes)
    └── contract/              # Contract tests (auth middleware)
```

---

## Prerequisites

Before you begin, ensure you have:

- **Python 3.12+** installed
- **[uv](https://github.com/astral-sh/uv)** package manager (recommended) or pip
- **Neon PostgreSQL** database with connection string
- **Qdrant Cloud** instance (or local Qdrant server)
- **API Keys**:
  - Gemini API key (for embeddings & AI)
  - Qdrant API key (if using Qdrant Cloud)

---

## Installation

### Option 1: Using uv (Recommended)

```bash
# Navigate to backend directory
cd backend

# Install dependencies (uv automatically creates venv)
uv sync

# Verify installation
uv run python --version  # Should show Python 3.12+
```

### Option 2: Using pip

```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python3.12 -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Verify installation
python --version
```

---

## Configuration

### Environment Variables

Create a `.env` file in the `backend/` directory with the following variables:

```bash
# Qdrant Vector Database
QDRANT_URL=https://your-cluster.aws.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key

# Gemini AI (for embeddings & title generation)
GEMINI_API_KEY=your_gemini_api_key
GEMINI_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/
GEMINI_MODEL_NAME=gemini-2.5-flash-lite

# Neon PostgreSQL Database
DATABASE_URL=postgresql://user:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require&channel_binding=require

# Better Auth JWT Validation
JWKS_URL=https://your-auth-server.vercel.app/api/auth/jwks
JWT_ISSUER=https://your-auth-server.vercel.app
JWT_AUDIENCE=https://your-auth-server.vercel.app
```

### Database Setup

1. **Create Neon Database**: Sign up at [Neon](https://neon.tech) and create a project
2. **Run Migrations**: Execute SQL scripts in order:

```bash
# Connect to Neon using psql or Neon SQL Editor
psql "postgresql://user:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require"

# Run migrations
\i scripts/migrations/003_create_personalization_tables.sql
\i scripts/migrations/004_create_translation_tables.sql
\i scripts/migrations/005_create_chat_tables.sql
```

3. **Verify Tables**:

```sql
-- Check created tables
\dt

-- Expected tables:
-- - chat_threads
-- - chat_thread_items
-- - personalization_history
-- - personalization_quota
-- - translation_history
-- - translation_quota
-- - user (from Better Auth)
```

### Qdrant Ingestion

Populate the vector database with textbook content:

```bash
# Ensure Qdrant is accessible
curl $QDRANT_URL/collections

# Run ingestion script (if available)
# Note: This script may need to be created based on your content structure
uv run python scripts/ingest_book.py
```

---

## Development

### Start Development Server

```bash
cd backend

# Using uv
uv run uvicorn main:app --reload --host 0.0.0.0 --port 8000

# Using pip (with activated venv)
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

Server will start at `http://localhost:8000`

### Interactive API Documentation

FastAPI auto-generates interactive docs:

- **Swagger UI**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc

### Health Check

```bash
curl http://localhost:8000/health

# Expected response:
# {
#   "status": "ok",
#   "service": "Physical AI & Robotics Platform",
#   "database": "connected"
# }
```

---

## API Documentation

### Core Endpoints

#### Health & Status

```http
GET /health
```
Returns service health status including database connectivity.

**Response:**
```json
{
  "status": "ok",
  "service": "Physical AI & Robotics Platform",
  "database": "connected"
}
```

---

#### RAG Tutor Chat

```http
POST /chatkit
Content-Type: application/json
Authorization: Bearer <optional_jwt_token>

{
  "action": "chat",
  "thread_id": "thread_abc123",
  "message": "Explain ROS 2 nodes"
}
```

**Features:**
- Streaming responses via Server-Sent Events (SSE)
- Automatic thread title generation
- Context-aware retrieval from knowledge base
- Supports authenticated and anonymous users

**Response (SSE Stream):**
```
data: {"type": "content", "delta": "ROS 2 nodes are..."}
data: {"type": "content", "delta": " fundamental building blocks"}
data: {"type": "done"}
```

---

#### User Profile Management

```http
# Create Profile
POST /profile
Authorization: Bearer <jwt_token>
Content-Type: application/json

{
  "user_id": "user_123",
  "age_range": "25_34",
  "education_level": "masters",
  "tech_background": "intermediate",
  "primary_goal": "career",
  "learning_mode": "hands_on",
  "learning_speed": "balanced",
  "time_per_week": "8_15",
  "preferred_language": "en"
}
```

**Response:** `201 Created`
```json
{
  "user_id": "user_123",
  "age_range": "25_34",
  ...
}
```

```http
# Get Profile
GET /profile
Authorization: Bearer <jwt_token>
```

**Response:** `200 OK` (same structure as POST)

```http
# Update Profile
PUT /profile
Authorization: Bearer <jwt_token>
Content-Type: application/json

{
  "user_id": "user_123",
  "learning_speed": "accelerated"
}
```

**Response:** `200 OK` (updated profile)

---

#### Content Personalization

```http
POST /personalize
Authorization: Bearer <jwt_token>
Content-Type: application/json

{
  "content": "Original technical content...",
  "profile": {
    "education_level": "bachelors",
    "tech_background": "beginner",
    "learning_mode": "visual"
  }
}
```

**Response:** `200 OK`
```json
{
  "personalized_content": "Adapted content for beginner visual learner...",
  "user_id": "user_123",
  "timestamp": "2025-12-17T10:30:00Z"
}
```

**Rate Limits:**
- 50 requests per day per user
- Quotas stored in `personalization_quota` table

---

#### Translation Service

```http
POST /translate
Authorization: Bearer <jwt_token>
Content-Type: application/json

{
  "text": "ROS 2 is a middleware framework",
  "target_language": "ur",
  "context": "robotics"
}
```

**Response:** `200 OK`
```json
{
  "translated_text": "ROS 2 ایک مڈل ویئر فریم ورک ہے",
  "source_language": "en",
  "target_language": "ur",
  "user_id": "user_123",
  "timestamp": "2025-12-17T10:35:00Z"
}
```

**Rate Limits:**
- 100 requests per day per user

---

### Authentication

The backend uses **JWT-based authentication** via Better Auth:

1. **JWT Token**: Obtained from Better Auth frontend flow
2. **Validation**: Backend validates token using JWKS endpoint
3. **User Context**: Extracted from `sub` claim in JWT
4. **Optional Auth**: RAG Tutor supports anonymous users (no JWT required)

**Protected Routes:**
- `/profile/*` - Requires valid JWT
- `/personalize` - Requires valid JWT
- `/translate` - Requires valid JWT

**Public Routes:**
- `/chatkit` - Optional JWT (enables persistence for authenticated users)
- `/health` - No auth required

---

## RAG System

### Architecture

The Retrieval-Augmented Generation (RAG) system combines:

1. **Vector Database (Qdrant)**: Stores embeddings of textbook content
2. **Embedding Model (Gemini text-embedding-004)**: Converts queries to vectors
3. **Semantic Search**: Finds top-K relevant passages
4. **LLM Synthesis (Gemini 2.5 Flash Lite)**: Generates contextual answers

### Knowledge Base Structure

```
qdrant_collection: "rag_tutor_knowledge_base"
├── Module 1: Core Learning Experience
│   ├── Week 1: Introduction to Physical AI
│   ├── Week 2: ROS 2 Fundamentals
│   └── Week 3: Robotics Mathematics
├── Module 2: Digital Twin (Isaac Sim)
│   ├── Week 4: Simulation Setup
│   ├── Week 5: Physics Engines
│   └── Week 6: Sensor Simulation
├── Module 3: AI-Robot Brain (Isaac AI)
│   ├── Week 7: Reinforcement Learning
│   ├── Week 8: Imitation Learning
│   └── Week 9: Sim-to-Real Transfer
└── Module 4: Vision-Language-Action (VLA)
    ├── Week 10: VLA Architectures
    ├── Week 11: Training VLA Models
    └── Week 12-13: Deployment & Projects
```

### Query Tool (`agent_tools.py`)

```python
@function_tool
def query_knowledge_base(args: KnowledgeQuery) -> str:
    """
    1. Generate embedding for user query
    2. Search Qdrant for top 3 similar chunks
    3. Return formatted results with sources
    """
```

### RAG Tutor Agent Behavior

**Persona:**
- Expert graduate-level robotics tutor
- Socratic method for conceptual questions
- Code explanation & debugging support
- Sim-to-Real gap awareness

**Tool Usage:**
- MUST use `query_knowledge_base` for textbook-specific questions
- ALWAYS cite sources from tool output
- Fallback to general knowledge if no relevant content found

**Example Interaction:**

**User:** "How do I create a ROS 2 publisher?"

**Agent Process:**
1. Calls `query_knowledge_base(query="ROS 2 publisher creation")`
2. Receives 3 relevant chunks from Week 2 content
3. Synthesizes answer with code example
4. Cites source: `[Week 2: ROS 2 Publishers & Subscribers](docs/module-1/week-2)`

---

## Database

### Schema Overview

#### `chat_threads`
Stores conversation metadata.

| Column | Type | Description |
|--------|------|-------------|
| `id` | VARCHAR(255) | Primary key (e.g., `thread_abc123`) |
| `user_id` | VARCHAR(255) | Foreign key to `user.id` (nullable) |
| `title` | VARCHAR(255) | Auto-generated thread title |
| `metadata` | JSONB | Arbitrary metadata (e.g., previous_response_id) |
| `created_at` | TIMESTAMPTZ | Thread creation timestamp |
| `updated_at` | TIMESTAMPTZ | Last activity timestamp |

**Indexes:**
- `idx_chat_threads_user_id` on `user_id`
- `idx_chat_threads_created_at` on `created_at DESC`

---

#### `chat_thread_items`
Stores individual messages and AI responses.

| Column | Type | Description |
|--------|------|-------------|
| `id` | VARCHAR(255) | Primary key (e.g., `message_xyz789`) |
| `thread_id` | VARCHAR(255) | Foreign key to `chat_threads.id` |
| `type` | VARCHAR(50) | Item type (user_message, assistant_message, etc.) |
| `role` | VARCHAR(50) | Message role (user, assistant, system) |
| `content` | JSONB | Full message content (max 32KB) |
| `created_at` | TIMESTAMPTZ | Item creation timestamp |
| `n_tokens` | INTEGER | Token count (for usage tracking) |

**Indexes:**
- `idx_chat_thread_items_thread_id` on `thread_id`
- `idx_chat_thread_items_thread_created` on `(thread_id, created_at)`

---

#### `personalization_quota`
Tracks usage limits for personalization feature.

| Column | Type | Description |
|--------|------|-------------|
| `user_id` | VARCHAR(255) | Foreign key to `user.id` |
| `requests_today` | INTEGER | Requests made today |
| `last_reset` | TIMESTAMPTZ | Last quota reset timestamp |
| `daily_limit` | INTEGER | Max requests per day (default: 50) |

---

#### `translation_quota`
Tracks usage limits for translation feature.

| Column | Type | Description |
|--------|------|-------------|
| `user_id` | VARCHAR(255) | Foreign key to `user.id` |
| `requests_today` | INTEGER | Requests made today |
| `last_reset` | TIMESTAMPTZ | Last quota reset timestamp |
| `daily_limit` | INTEGER | Max requests per day (default: 100) |

---

### Connection Pooling

The backend uses **asyncpg connection pooling** for efficient Neon connectivity:

```python
# In NeonChatKitStore
pool = await asyncpg.create_pool(
    connection_string,
    min_size=2,
    max_size=10,
    command_timeout=60
)
```

**Configuration:**
- Min connections: 2
- Max connections: 10
- Command timeout: 60 seconds
- SSL/TLS: Required (enforced by Neon)

---

## Testing

### Run All Tests

```bash
cd backend

# Using uv
uv run pytest

# Using pip (with activated venv)
pytest
```

### Test Structure

```
tests/
├── unit/
│   ├── models/
│   │   └── test_profile_model.py      # Pydantic model validation
│   ├── services/
│   │   ├── test_jwt_service.py        # JWT validation logic
│   │   └── test_database.py           # Connection pool tests
│   └── test_agent_tools.py            # RAG tool unit tests
│
├── integration/
│   ├── api/
│   │   ├── test_profile_routes.py     # Profile API integration
│   │   └── test_chatkit.py            # ChatKit endpoint tests
│   └── test_health.py                 # Health check integration
│
└── contract/
    └── test_auth_middleware.py        # JWT contract validation
```

### Run Specific Test Suites

```bash
# Unit tests only
uv run pytest tests/unit/

# Integration tests (requires DATABASE_URL)
uv run pytest tests/integration/

# Single test file
uv run pytest tests/unit/test_database.py

# With coverage report
uv run pytest --cov=src --cov-report=html
```

### Test Environment Variables

For testing, create a `.env.test` file:

```bash
# Test database (separate from production)
DATABASE_URL=postgresql://test_user:test_pass@localhost:5432/test_db

# Test Qdrant (can be empty collection)
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=

# Test Gemini API (use actual key for integration tests)
GEMINI_API_KEY=your_test_key
```

---

## Deployment

### Vercel Deployment

The backend is configured for Vercel serverless deployment via `vercel.json`:

```json
{
  "version": 2,
  "builds": [
    {
      "src": "main.py",
      "use": "@vercel/python"
    }
  ],
  "routes": [
    {
      "src": "/(.*)",
      "dest": "main.py"
    }
  ]
}
```

**Deployment Steps:**

1. **Install Vercel CLI**:
```bash
npm install -g vercel
```

2. **Configure Environment Variables** in Vercel Dashboard:
   - `DATABASE_URL`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
   - `GEMINI_API_KEY`
   - `JWKS_URL`
   - `JWT_ISSUER`
   - `JWT_AUDIENCE`

3. **Deploy**:
```bash
cd backend
vercel --prod
```

4. **Update CORS Origins** in `main.py`:
```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://your-frontend.vercel.app",
        "https://your-auth.vercel.app",
    ],
    # ...
)
```

---

### Docker Deployment (Alternative)

Create `Dockerfile` in `backend/`:

```dockerfile
FROM python:3.12-slim

WORKDIR /app

# Install uv
RUN pip install uv

# Copy dependency files
COPY pyproject.tomllll requirements.txt ./

# Install dependencies
RUN uv pip install --system -r requirements.txt

# Copy application code
COPY . .

# Expose port
EXPOSE 8000

# Run application
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

**Build & Run:**
```bash
docker build -t robotics-backend .
docker run -p 8000:8000 --env-file .env robotics-backend
```

---

### Production Checklist

- [ ] Set all environment variables in deployment platform
- [ ] Run database migrations in production Neon instance
- [ ] Configure CORS origins for production domains
- [ ] Enable HTTPS/TLS for all endpoints
- [ ] Set up monitoring (e.g., Sentry for error tracking)
- [ ] Configure rate limiting middleware (if not using API gateway)
- [ ] Test JWT validation with production Better Auth JWKS
- [ ] Verify Qdrant collection exists and is populated
- [ ] Set up database backups (Neon provides automatic backups)
- [ ] Configure logging (structured JSON logs for production)

---

## Troubleshooting

### Database Connection Issues

**Problem:** `OperationalError: could not connect to server`

**Solution:**
1. Verify `DATABASE_URL` is correct and includes `?sslmode=require`
2. Check Neon dashboard for database status
3. Ensure IP is allowlisted (Neon allows all by default)
4. Test connection with `psql`:
```bash
psql "$DATABASE_URL"
```

---

### Qdrant Connection Errors

**Problem:** `Error searching knowledge base`

**Solution:**
1. Verify `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
2. Check Qdrant Cloud cluster status
3. Test connection:
```bash
curl -H "api-key: $QDRANT_API_KEY" $QDRANT_URL/collections
```
4. Ensure collection `rag_tutor_knowledge_base` exists

---

### JWT Validation Failures

**Problem:** `401 Unauthorized` on protected routes

**Solution:**
1. Verify `JWKS_URL`, `JWT_ISSUER`, `JWT_AUDIENCE` match Better Auth config
2. Test JWKS endpoint:
```bash
curl https://your-auth.vercel.app/api/auth/jwks
```
3. Check JWT token expiration (`exp` claim)
4. Validate token manually at [jwt.io](https://jwt.io)

---

### RAG Agent Not Citing Sources

**Problem:** Agent provides generic answers without textbook references

**Solution:**
1. Check if Qdrant collection is populated:
```python
from qdrant_client import QdrantClient
client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
client.get_collection("rag_tutor_knowledge_base")
# Should show non-zero `points_count`
```
2. Test embedding generation:
```python
from google import genai
client = genai.Client(api_key=GEMINI_API_KEY)
response = client.models.embed_content(
    model="text-embedding-004",
    contents="test query"
)
print(response.embeddings[0].values[:5])  # Should print vector
```
3. Review agent logs for tool call errors

---

### Chat History Not Persisting

**Problem:** Conversations lost after page reload

**Solution:**
1. Verify `chat_threads` and `chat_thread_items` tables exist:
```sql
SELECT COUNT(*) FROM chat_threads;
```
2. Check `user_id` in context is being passed correctly:
```python
# In main.py /chatkit endpoint
context = {"user_id": user.id} if user else {}
```
3. Review `NeonChatKitStore` logs for save/load errors
4. Ensure `thread_id` is being sent from frontend in subsequent requests

---

### Performance Issues

**Problem:** Slow API responses

**Solution:**
1. **Database Queries**: Check connection pool size (increase `max_size` if needed)
2. **Qdrant Latency**: Use Qdrant Cloud region closest to backend deployment
3. **Embedding Generation**: Cache embeddings for common queries
4. **LLM Response Time**: Consider using faster model (e.g., `gemini-2.5-flash`)
5. **Enable Caching**: Add Redis for session/profile caching

---

## Contributing

This backend follows the project's **Spec-Driven Development (SDD)** workflow:

1. **Feature Specs**: Review `/specs` directory for active features
2. **TDD Mandate**: Write tests before implementation (Red → Green → Refactor)
3. **Type Safety**: All functions must have Pydantic models or type hints
4. **Async/Await**: Use async patterns for all I/O operations
5. **Error Handling**: Raise `HTTPException` with clear status codes and messages

### Code Style

- **Formatting**: Black (line length 100)
- **Linting**: Ruff
- **Type Checking**: mypy (strict mode)
- **Docstrings**: Google-style docstrings for all public functions

---

## License

This project is part of the Physical AI & Humanoid Robotics learning platform. See root `LICENSE` file for details.

---

## Support

For technical questions:
- Review API documentation at `/docs` endpoint
- Check troubleshooting section above
- Create an issue in the project repository

For course-related questions:
- Use the RAG Tutor chatbot at `/chatkit`
- Consult the textbook content in `/website/docs`

---

**Built with:** FastAPI, OpenAI Agents, Qdrant, Neon PostgreSQL, and Gemini AI

**Version:** 1.0.0 | **Last Updated:** December 2025
