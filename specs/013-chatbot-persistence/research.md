# Research: ChatKit Persistence with Neon Database

## Summary

This document captures research findings from Context7 on how to implement persistent storage for OpenAI ChatKit using Neon PostgreSQL database.

## ChatKit Store Interface

The ChatKit framework provides an abstract `Store` class that must be implemented for data persistence. The current implementation uses a file-based store (`chatkit_store.json`), but this can be replaced with a database-backed implementation.

### Store Abstract Base Class

```python
from abc import ABC
from typing import Generic, Literal, TypeVar

from chatkit.models import Attachment, Page, ThreadItem, ThreadMetadata

TContext = TypeVar("TContext")

class Store(ABC, Generic[TContext]):
    def generate_thread_id(self, context: TContext) -> str: ...

    def generate_item_id(
        self,
        item_type: Literal["message", "tool_call", "task", "workflow", "attachment"],
        thread: ThreadMetadata,
        context: TContext,
    ) -> str: ...

    async def load_thread(self, thread_id: str, context: TContext) -> ThreadMetadata: ...

    async def save_thread(self, thread: ThreadMetadata, context: TContext) -> None: ...

    async def load_thread_items(
        self,
        thread_id: str,
        after: str | None,
        limit: int,
        order: str,
        context: TContext,
    ) -> Page[ThreadItem]: ...

    async def load_threads(
        self,
        limit: int,
        after: str | None,
        order: str,
        context: TContext,
    ) -> Page[ThreadMetadata]: ...

    async def add_thread_item(
        self, thread_id: str, item: ThreadItem, context: TContext
    ) -> None: ...

    async def save_item(self, thread_id: str, item: ThreadItem, context: TContext) -> None: ...

    async def load_item(self, thread_id: str, item_id: str, context: TContext) -> ThreadItem: ...

    async def delete_thread(self, thread_id: str, context: TContext) -> None: ...

    async def delete_thread_item(
        self, thread_id: str, item_id: str, context: TContext
    ) -> None: ...

    # Attachment methods
    async def save_attachment(self, attachment: Attachment, context: TContext) -> None: ...
    async def load_attachment(self, attachment_id: str, context: TContext) -> Attachment: ...
    async def delete_attachment(self, attachment_id: str, context: TContext) -> None: ...
```

### Key Implementation Notes

1. **JSON Column Storage**: The official recommendation is to serialize models to JSON-typed columns for forward compatibility across library versions.

2. **ThreadMetadata**: Contains thread-level information and a `metadata` dictionary for storing arbitrary data (e.g., `previous_response_id` for Agent SDK optimization).

3. **ThreadItem**: Represents individual messages with types: `message`, `tool_call`, `task`, `workflow`, `attachment`.

4. **Pagination**: The `Page` type supports cursor-based pagination with `after`, `limit`, and `has_more` fields.

## Neon Database Schema Recommendations

Based on Context7 research and ChatKit requirements:

### Threads Table

```sql
CREATE TABLE chat_threads (
    id VARCHAR(255) PRIMARY KEY,
    user_id VARCHAR(255),  -- NULL for anonymous users
    session_id VARCHAR(255),  -- For anonymous user identification
    metadata JSONB DEFAULT '{}',
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_chat_threads_user_id ON chat_threads(user_id);
CREATE INDEX idx_chat_threads_session_id ON chat_threads(session_id);
CREATE INDEX idx_chat_threads_created_at ON chat_threads(created_at DESC);
```

### Thread Items (Messages) Table

```sql
CREATE TABLE chat_thread_items (
    id VARCHAR(255) PRIMARY KEY,
    thread_id VARCHAR(255) NOT NULL REFERENCES chat_threads(id) ON DELETE CASCADE,
    type VARCHAR(50) NOT NULL,  -- 'message', 'tool_call', 'task', 'workflow', 'attachment'
    role VARCHAR(50),  -- 'user', 'assistant', 'system'
    content JSONB NOT NULL,  -- Full item serialized as JSON
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    n_tokens INTEGER  -- Optional: for token counting/limits
);

CREATE INDEX idx_chat_thread_items_thread_id ON chat_thread_items(thread_id);
CREATE INDEX idx_chat_thread_items_created_at ON chat_thread_items(thread_id, created_at);
```

### Attachments Table (Optional)

```sql
CREATE TABLE chat_attachments (
    id VARCHAR(255) PRIMARY KEY,
    thread_id VARCHAR(255) REFERENCES chat_threads(id) ON DELETE CASCADE,
    mime_type VARCHAR(255) NOT NULL,
    name VARCHAR(255),
    size_bytes BIGINT,
    storage_url TEXT,  -- Reference to blob storage (S3, etc.)
    metadata JSONB DEFAULT '{}',
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);
```

## Implementation Strategy

### 1. Create NeonChatKitStore Class

Implement the `Store` interface with async database operations:

```python
from chatkit.store import Store
from chatkit.types import ThreadMetadata, ThreadItem, Page

class NeonChatKitStore(Store[dict]):
    def __init__(self, connection_string: str):
        self.connection_string = connection_string

    async def load_thread(self, thread_id: str, context: dict) -> ThreadMetadata:
        # Query chat_threads table
        # Deserialize JSONB metadata
        # Return ThreadMetadata instance
        pass

    async def save_thread(self, thread: ThreadMetadata, context: dict) -> None:
        # Upsert into chat_threads table
        # Serialize metadata to JSONB
        pass

    # ... implement all other methods
```

### 2. Use Connection Pooling

For serverless Neon, use connection pooling:

```python
import asyncpg

class NeonChatKitStore(Store[dict]):
    def __init__(self, connection_string: str):
        self._pool = None
        self.connection_string = connection_string

    async def get_pool(self):
        if self._pool is None:
            self._pool = await asyncpg.create_pool(self.connection_string)
        return self._pool
```

### 3. Handle Pagination

Implement cursor-based pagination for `load_threads` and `load_thread_items`:

```python
async def load_thread_items(
    self,
    thread_id: str,
    after: str | None,
    limit: int,
    order: str,
    context: dict,
) -> Page[ThreadItem]:
    pool = await self.get_pool()
    async with pool.acquire() as conn:
        if after:
            # Get the created_at of the cursor item
            cursor_row = await conn.fetchrow(
                "SELECT created_at FROM chat_thread_items WHERE id = $1",
                after
            )
            if order == "desc":
                query = """
                    SELECT * FROM chat_thread_items
                    WHERE thread_id = $1 AND created_at < $2
                    ORDER BY created_at DESC
                    LIMIT $3
                """
            else:
                query = """
                    SELECT * FROM chat_thread_items
                    WHERE thread_id = $1 AND created_at > $2
                    ORDER BY created_at ASC
                    LIMIT $3
                """
            rows = await conn.fetch(query, thread_id, cursor_row['created_at'], limit + 1)
        else:
            # No cursor, start from beginning
            rows = await conn.fetch(
                f"SELECT * FROM chat_thread_items WHERE thread_id = $1 ORDER BY created_at {'DESC' if order == 'desc' else 'ASC'} LIMIT $2",
                thread_id, limit + 1
            )

        has_more = len(rows) > limit
        data = [self._row_to_thread_item(row) for row in rows[:limit]]

        return Page(
            data=data,
            has_more=has_more,
            after=data[-1].id if has_more and data else None
        )
```

## Migration Strategy

### From File-Based to Database

1. **Read existing data** from `chatkit_store.json`
2. **Create database tables** using migration script
3. **Insert existing threads and items** into database
4. **Switch store implementation** in `chatkit_server.py`
5. **Remove file-based storage** after validation

### Migration Script Outline

```python
import json
import asyncpg

async def migrate_chatkit_to_neon():
    # Load existing data
    with open("chatkit_store.json", "r") as f:
        data = json.load(f)

    pool = await asyncpg.create_pool(os.getenv("DATABASE_URL"))

    async with pool.acquire() as conn:
        # Migrate threads
        for thread_id, thread_data in data.get("threads", {}).items():
            await conn.execute("""
                INSERT INTO chat_threads (id, metadata, created_at, updated_at)
                VALUES ($1, $2, $3, $4)
                ON CONFLICT (id) DO NOTHING
            """, thread_id, json.dumps(thread_data.get("metadata", {})),
                thread_data["created_at"], thread_data.get("updated_at"))

        # Migrate items
        for thread_id, items in data.get("items", {}).items():
            for item in items:
                await conn.execute("""
                    INSERT INTO chat_thread_items (id, thread_id, type, role, content, created_at)
                    VALUES ($1, $2, $3, $4, $5, $6)
                    ON CONFLICT (id) DO NOTHING
                """, item["id"], thread_id, item["type"],
                    item.get("role"), json.dumps(item), item["created_at"])
```

## References

- Context7: `/openai/chatkit-python` - ChatKit Python SDK
- Context7: `/websites/openai_github_io_chatkit-python` - ChatKit Python Documentation
- Context7: `/openai/openai-chatkit-advanced-samples` - Advanced ChatKit Examples
- Context7: `/llmstxt/neon_tech-llms.txt` - Neon Database Documentation
