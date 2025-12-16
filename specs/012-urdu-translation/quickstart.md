# Quickstart: Urdu Translation Feature

**Feature**: 012-urdu-translation
**Date**: 2025-12-15

## Prerequisites

- Node.js >= 20.0
- Python >= 3.12
- uv (Python package manager)
- PostgreSQL (Neon) database access
- OpenAI API key (for Gemini model via openai-agents)

## Setup

### 1. Backend Setup

```bash
cd backend

# Install dependencies
uv sync

# Set environment variables
export DATABASE_URL="postgresql://..."
export OPENAI_API_KEY="sk-..."

# Run database migration
uv run python -c "
from scripts.migrations import run_migration
run_migration('004_create_translation_tables.sql')
"

# Start development server
uv run uvicorn main:app --reload --port 8000
```

### 2. Frontend Setup

```bash
cd website

# Install dependencies
npm install

# Start development server
npm start
```

### 3. Verify Installation

```bash
# Check backend health
curl http://localhost:8000/health

# Check translation quota endpoint
curl http://localhost:8000/api/translate/quota

# Test translation (requires running backend)
curl -X POST http://localhost:8000/api/translate/urdu \
  -H "Content-Type: application/json" \
  -d '{
    "page_content": "# Hello World\n\nThis is a test.",
    "page_url": "/test"
  }'
```

## Development Workflow

### Running Tests

```bash
# Backend unit tests
cd backend
uv run pytest tests/unit/services/test_translation_service.py -v

# Backend integration tests
uv run pytest tests/integration/api/test_translation_api.py -v

# Frontend unit tests
cd website
npm test -- --testPathPattern=translation

# E2E tests
npm run test:e2e -- translation.spec.ts
```

### Test-Driven Development (TDD)

Per the project constitution, follow Red-Green-Refactor:

1. **Red**: Write a failing test first
2. **Green**: Write minimal code to pass the test
3. **Refactor**: Clean up while keeping tests green

Example TDD flow for translation service:

```python
# 1. RED - Write failing test
def test_translate_preserves_code_blocks():
    content = "Text\n```python\nprint('hello')\n```\nMore text"
    result = translation_service.translate(content)
    assert "```python" in result
    assert "print('hello')" in result

# 2. GREEN - Implement minimal solution
# 3. REFACTOR - Improve implementation
```

## Key Files to Modify

### Backend

| File | Purpose |
|------|---------|
| `backend/src/api/translation/routes.py` | API endpoints |
| `backend/src/services/translation_service.py` | Business logic |
| `backend/src/ai_agents/translation_agent.py` | Gemini agent |
| `backend/src/models/translation.py` | Pydantic models |

### Frontend

| File | Purpose |
|------|---------|
| `website/src/components/translation/TranslationBar.tsx` | UI component |
| `website/src/hooks/useTranslation.ts` | State management |
| `website/src/services/translationService.ts` | API client + cache |
| `website/src/types/translation.ts` | TypeScript types |
| `website/src/theme/Heading/index.tsx` | Render TranslationBar |

## API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/api/translate/urdu` | Translate content to Urdu |
| GET | `/api/translate/quota` | Get quota status |
| GET | `/api/translate/history` | Check translation history |

## Testing the Feature

### Manual Testing Checklist

1. **Basic Translation**
   - [ ] Navigate to any doc page (e.g., `/docs/module-01/embodied-intelligence`)
   - [ ] Click "Translate to Urdu" button
   - [ ] Verify loading state appears
   - [ ] Verify translated content displays (RTL direction)
   - [ ] Verify code blocks are preserved (LTR)

2. **Toggle View**
   - [ ] After translation, click "Show Original"
   - [ ] Verify original English content appears
   - [ ] Click "Show Translated"
   - [ ] Verify cached translation appears instantly

3. **Quota Management**
   - [ ] Translate 5 pages
   - [ ] Verify quota counter updates
   - [ ] On 6th attempt, verify quota exceeded message

4. **Cache Behavior**
   - [ ] Translate a page
   - [ ] Navigate away and return
   - [ ] Verify "View Translated" option appears
   - [ ] Verify cached version loads without API call

5. **Anonymous vs Authenticated**
   - [ ] Test translation without logging in
   - [ ] Log in and verify quota is separate

## Troubleshooting

### Common Issues

**Issue**: "Translation failed" error
- Check OpenAI API key is set
- Verify backend is running on port 8000
- Check backend logs for Gemini API errors

**Issue**: RTL text not displaying correctly
- Verify CSS direction is applied
- Check for conflicting Tailwind classes
- Ensure Urdu font is available

**Issue**: Quota not tracking
- Verify database connection
- Check if migration ran successfully
- Look for errors in translation_quota table

### Debug Mode

```bash
# Enable verbose logging in backend
export LOG_LEVEL=DEBUG
uv run uvicorn main:app --reload

# Enable React DevTools in frontend
npm start  # Development mode has DevTools enabled
```

## Architecture Reference

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   TranslationBar │────>│  useTranslation │────>│ translationService│
│   (Component)    │     │  (Hook)         │     │ (API Client)    │
└─────────────────┘     └─────────────────┘     └─────────────────┘
                                                         │
                                                         ▼
                                                ┌─────────────────┐
                                                │  FastAPI Backend │
                                                │  /api/translate  │
                                                └─────────────────┘
                                                         │
                                                         ▼
                                                ┌─────────────────┐
                                                │  translation_    │
                                                │  service.py      │
                                                └─────────────────┘
                                                         │
                                         ┌───────────────┴───────────────┐
                                         ▼                               ▼
                                ┌─────────────────┐             ┌─────────────────┐
                                │  urdu_translation│             │  Neon Database  │
                                │  _agent (Gemini) │             │  (quota/history)│
                                └─────────────────┘             └─────────────────┘
```
