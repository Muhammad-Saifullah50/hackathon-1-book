# Quickstart: Page Content Personalization

**Feature**: 011-personalize-page
**Date**: 2025-12-14

## Prerequisites

- Python 3.12+ with `uv` package manager
- Node.js 20+ with npm
- Neon PostgreSQL database (configured)
- OpenAI API key with access to GPT-4

## Environment Setup

### 1. Backend Environment Variables

Add to `backend/.env`:

```bash
# Existing variables
OPENAI_API_KEY=sk-...
DATABASE_URL=postgresql://...@neon.tech/...

# New for personalization (optional - defaults shown)
PERSONALIZATION_DAILY_LIMIT=5
PERSONALIZATION_TIMEOUT_SECONDS=15
```

### 2. Run Database Migration

```bash
cd backend

# Apply the personalization tables migration
psql $DATABASE_URL -f scripts/migrations/003_create_personalization_tables.sql
```

Verify tables created:
```sql
SELECT table_name FROM information_schema.tables
WHERE table_name IN ('personalization_history', 'personalization_quota');
```

### 3. Install Backend Dependencies

```bash
cd backend
uv sync  # Updates dependencies including openai-agents
```

### 4. Install Frontend Dependencies

```bash
cd website
npm install
```

## Running the Application

### Start Backend

```bash
cd backend
uv run uvicorn main:app --reload --port 8000
```

### Start Frontend

```bash
cd website
npm start
```

Access at: http://localhost:3000

## Testing the Feature

### 1. Manual Testing Flow

1. **Login**: Sign in with a test account that has a completed profile
2. **Navigate**: Go to any documentation page (e.g., `/docs/module-1/intro`)
3. **Personalize**: Click the "Personalize Page" button in the PersonalizationBar
4. **Wait**: Loading indicator should appear (up to 15 seconds)
5. **Verify**: Content should be personalized based on your profile
6. **Toggle**: Click "Show Original" to see the original content
7. **Return**: Navigate away and back - should see "View Personalized" option

### 2. API Testing with cURL

**Get Quota Status:**
```bash
curl -X GET http://localhost:8000/api/personalization/quota \
  -H "Authorization: Bearer <your-jwt-token>"
```

**Personalize Content:**
```bash
curl -X POST http://localhost:8000/api/personalization/personalize \
  -H "Authorization: Bearer <your-jwt-token>" \
  -H "Content-Type: application/json" \
  -d '{
    "pageUrl": "/docs/module-1/intro",
    "pageContent": "# Introduction\n\nRobotics combines...",
    "isFreeRepersonalization": false
  }'
```

**Get History:**
```bash
curl -X GET http://localhost:8000/api/personalization/history \
  -H "Authorization: Bearer <your-jwt-token>"
```

### 3. Running Automated Tests

**Backend Unit Tests:**
```bash
cd backend
uv run pytest tests/unit/services/test_personalization_service.py -v
```

**Backend Integration Tests:**
```bash
cd backend
uv run pytest tests/integration/api/test_personalization_api.py -v
```

**Frontend Unit Tests:**
```bash
cd website

# Run all personalization-related tests
npm test -- --testPathPattern="personalization|usePersonalization|PersonalizationBar"

# Run specific test files
npm test -- website/tests/unit/hooks/usePersonalization.test.ts
npm test -- website/tests/unit/components/profile/PersonalizationBar.test.tsx
```

**E2E Tests:**
```bash
cd website

# Run all E2E personalization tests
npx playwright test tests/e2e/personalization.spec.ts

# Run with headed browser for debugging
npx playwright test tests/e2e/personalization.spec.ts --headed

# Run specific test
npx playwright test -g "Full Personalization Flow"
```

## Feature Verification Checklist

### Core Functionality
- [ ] Personalize button visible for logged-in users with profile
- [ ] Loading state appears during personalization
- [ ] Personalized content displays correctly
- [ ] Toggle between original and personalized works
- [ ] Code blocks preserved unchanged
- [ ] Images and diagrams preserved

### Quota System
- [ ] Quota decrements on personalization
- [ ] Quota displayed in UI
- [ ] Error shown when quota exceeded
- [ ] Quota resets at midnight UTC

### Caching
- [ ] Content cached in localStorage after personalization
- [ ] Cached content loads instantly on page return
- [ ] Cache invalidates on profile change

### Cross-Device
- [ ] History API returns personalized pages
- [ ] "Re-personalize (Free)" shown on new device
- [ ] Free re-personalization doesn't decrement quota

## Troubleshooting

### "Profile incomplete" error
- User needs to complete the signup wizard at `/signup-wizard`
- Verify profile exists: Check `user_profile` table for user_id

### Personalization times out
- Check OpenAI API status
- Verify OPENAI_API_KEY is valid
- Check content size (very long pages may timeout)

### Quota not resetting
- Verify server timezone is UTC
- Check `personalization_quota` table for date entries
- Quota resets at midnight UTC, not local time

### Cache not working
- Check browser localStorage (Developer Tools > Application > Local Storage)
- Verify key format: `personalized_<pageUrl>_<userId>`
- Check for localStorage quota errors in console

## Key Files

| File | Purpose |
|------|---------|
| `backend/src/ai_agents/personalization_agent.py` | Gemini AI Agent with dynamic instructions |
| `backend/src/services/personalization_service.py` | Business logic for personalization |
| `backend/src/api/personalization/routes.py` | API endpoints |
| `backend/src/models/personalization.py` | Pydantic request/response models |
| `website/src/hooks/usePersonalization.ts` | React hook for personalization state |
| `website/src/services/personalizationService.ts` | API client + localStorage cache |
| `website/src/components/profile/PersonalizationBar.tsx` | UI component |
| `website/src/types/personalization.ts` | TypeScript type definitions |
| `website/src/utils/hashUtils.ts` | Profile and content hashing utilities |

## Test Files

| File | Purpose |
|------|---------|
| `backend/tests/unit/services/test_personalization_service.py` | Unit tests for service layer |
| `backend/tests/integration/api/test_personalization_api.py` | Integration tests for API |
| `website/tests/unit/hooks/usePersonalization.test.ts` | Hook unit tests |
| `website/tests/unit/components/profile/PersonalizationBar.test.tsx` | Component tests |
| `website/tests/e2e/personalization.spec.ts` | E2E tests (Playwright) |

## Development Tips

1. **Testing without OpenAI costs**: Set up a mock agent that returns modified content with a prefix like "[PERSONALIZED] " for development testing.

2. **Debugging dynamic instructions**: Log the generated instructions in `personalization_agent.py` to verify profile attributes are being used correctly.

3. **Cache inspection**: Use browser DevTools to inspect localStorage entries and verify cache behavior.

4. **Quota bypass for testing**: Temporarily increase `PERSONALIZATION_DAILY_LIMIT` in `.env` for testing.
