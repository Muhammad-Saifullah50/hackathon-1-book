# Quickstart: Chatbot Persistence

**Feature**: 013-chatbot-persistence
**Created**: 2025-12-16

## Prerequisites

- Neon PostgreSQL database configured (DATABASE_URL in `.env`)
- Backend server running (`cd backend && uv run uvicorn main:app --reload`)
- Frontend dev server running (`cd website && npm start`)
- User authentication working (from feature 010)

## Setup Steps

### 1. Run Database Migration

```bash
cd backend

# Run the migration script
uv run python -c "
import asyncio
import asyncpg
import os
from dotenv import load_dotenv

load_dotenv()

async def migrate():
    conn = await asyncpg.connect(os.getenv('DATABASE_URL'))

    # Read and execute migration
    with open('scripts/migrations/005_create_chat_tables.sql', 'r') as f:
        await conn.execute(f.read())

    print('Migration completed successfully')
    await conn.close()

asyncio.run(migrate())
"
```

### 2. Migrate Existing Data (if any)

```bash
cd backend

# Run migration script for existing chatkit_store.json
uv run python scripts/migrate_chatkit_to_neon.py
```

### 3. Verify Setup

```bash
# Check tables exist
uv run python -c "
import asyncio
import asyncpg
import os
from dotenv import load_dotenv

load_dotenv()

async def verify():
    conn = await asyncpg.connect(os.getenv('DATABASE_URL'))

    # Check tables
    tables = await conn.fetch('''
        SELECT table_name FROM information_schema.tables
        WHERE table_schema = 'public' AND table_name LIKE 'chat_%'
    ''')

    for t in tables:
        print(f'✓ Table exists: {t[\"table_name\"]}')

    await conn.close()

asyncio.run(verify())
"
```

## Testing the Feature

### Test 1: Create a New Chat (Authenticated User)

1. Log in to the application
2. Open the chatbot widget
3. Send a message: "Hello, I'm testing chat persistence"
4. Wait for AI response
5. Close the browser tab
6. Reopen the application and log in
7. Open chatbot - **Expected**: Previous conversation visible

### Test 2: Verify Database Persistence

```bash
cd backend

uv run python -c "
import asyncio
import asyncpg
import os
from dotenv import load_dotenv

load_dotenv()

async def check_messages():
    conn = await asyncpg.connect(os.getenv('DATABASE_URL'))

    threads = await conn.fetch('SELECT * FROM chat_threads LIMIT 5')
    print(f'Threads: {len(threads)}')

    items = await conn.fetch('SELECT * FROM chat_thread_items LIMIT 10')
    print(f'Items: {len(items)}')

    for item in items:
        print(f'  - {item[\"type\"]}: {item[\"role\"]} at {item[\"created_at\"]}')

    await conn.close()

asyncio.run(check_messages())
"
```

### Test 3: Anonymous User (No Persistence)

1. Open the application in incognito/private window (not logged in)
2. Open chatbot and send a message
3. Close the browser
4. Reopen - **Expected**: No previous conversation (fresh start)

### Test 4: Multiple Threads

1. Log in to the application
2. Start a chat, send a few messages
3. Click "New Chat" to create a new thread
4. Send messages in the new thread
5. Switch back to the first thread
6. **Expected**: Each thread maintains its own history

### Test 5: Offline Resilience

1. Log in and open chatbot
2. Disable network (DevTools > Network > Offline)
3. Send a message
4. **Expected**: Message appears with pending indicator
5. Re-enable network
6. **Expected**: Pending indicator disappears, message persisted

## Running Tests

### Unit Tests

```bash
cd backend
uv run pytest tests/unit/services/test_chat_store.py -v
```

### Integration Tests

```bash
cd backend
uv run pytest tests/integration/api/test_chat_persistence.py -v
```

### E2E Tests

```bash
cd website
npm run test:e2e -- --grep "chat persistence"
```

## Troubleshooting

### "Connection refused" errors

- Verify DATABASE_URL is correct in `.env`
- Check Neon database is accessible
- Ensure connection pooling is enabled

### Messages not persisting

- Check browser console for errors
- Verify user is authenticated (not anonymous)
- Check backend logs for database errors

### Slow load times

- Verify indexes exist on tables
- Check for N+1 query issues
- Review connection pool settings

## Environment Variables

```env
# Required in backend/.env
DATABASE_URL=postgresql://user:pass@host.neon.tech/dbname?sslmode=require

# Optional
CHAT_POOL_MIN_SIZE=2
CHAT_POOL_MAX_SIZE=10
```

## Key Files

| File | Purpose |
|------|---------|
| `backend/src/services/chat_store.py` | NeonChatKitStore implementation |
| `backend/scripts/migrations/005_create_chat_tables.sql` | Database schema |
| `backend/chatkit_server.py` | ChatKit server with new store |
| `website/src/components/ChatWidget.tsx` | Chat UI with pending indicator |
