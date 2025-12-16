# Data Model: Chatbot Persistence

**Feature**: 013-chatbot-persistence
**Created**: 2025-12-16

## Entity Relationship Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                         User                                 │
│  (from auth system - not created by this feature)           │
└─────────────────────────────────────────────────────────────┘
                              │
                              │ 1:N (user owns threads)
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                      chat_threads                            │
├─────────────────────────────────────────────────────────────┤
│ id: VARCHAR(255) [PK]                                        │
│ user_id: VARCHAR(255) [FK, nullable]                        │
│ title: VARCHAR(255) [nullable]                               │
│ metadata: JSONB                                              │
│ created_at: TIMESTAMPTZ                                      │
│ updated_at: TIMESTAMPTZ                                      │
└─────────────────────────────────────────────────────────────┘
                              │
                              │ 1:N (thread contains items)
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    chat_thread_items                         │
├─────────────────────────────────────────────────────────────┤
│ id: VARCHAR(255) [PK]                                        │
│ thread_id: VARCHAR(255) [FK]                                │
│ type: VARCHAR(50)                                            │
│ role: VARCHAR(50) [nullable]                                 │
│ content: JSONB                                               │
│ created_at: TIMESTAMPTZ                                      │
│ n_tokens: INTEGER [nullable]                                 │
└─────────────────────────────────────────────────────────────┘
```

## Entities

### ChatThread

Represents a conversation session containing multiple messages.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | VARCHAR(255) | PK, NOT NULL | Unique thread identifier (format: `thread_{uuid}`) |
| user_id | VARCHAR(255) | FK → users.id, NULL | Owner user ID; NULL only during anonymous sessions |
| title | VARCHAR(255) | NULL | Optional thread title (auto-generated from first message if not set) |
| metadata | JSONB | DEFAULT '{}' | Arbitrary metadata (e.g., `previous_response_id` for Agent SDK) |
| created_at | TIMESTAMPTZ | NOT NULL, DEFAULT NOW() | Thread creation timestamp |
| updated_at | TIMESTAMPTZ | NOT NULL, DEFAULT NOW() | Last update timestamp (updated on new messages) |

**Indexes**:
- `idx_chat_threads_user_id` on `user_id` - For loading user's threads
- `idx_chat_threads_created_at` on `created_at DESC` - For pagination/sorting

**Validation Rules**:
- `id` must match pattern `thread_[a-z0-9]+`
- `user_id` must reference valid user when not NULL

### ChatThreadItem

Represents a single message or item within a thread.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | VARCHAR(255) | PK, NOT NULL | Unique item identifier (format: `{type}_{uuid}`) |
| thread_id | VARCHAR(255) | FK → chat_threads.id, NOT NULL | Parent thread reference |
| type | VARCHAR(50) | NOT NULL | Item type: `message`, `tool_call`, `task`, `workflow`, `attachment` |
| role | VARCHAR(50) | NULL | Message role: `user`, `assistant`, `system` (NULL for non-message types) |
| content | JSONB | NOT NULL | Full item content serialized as JSON (max 32KB) |
| created_at | TIMESTAMPTZ | NOT NULL, DEFAULT NOW() | Item creation timestamp |
| n_tokens | INTEGER | NULL | Token count for content (optional, for analytics) |

**Indexes**:
- `idx_chat_thread_items_thread_id` on `thread_id` - For loading thread items
- `idx_chat_thread_items_thread_created` on `(thread_id, created_at)` - For paginated loading

**Validation Rules**:
- `type` must be one of: `message`, `tool_call`, `task`, `workflow`, `attachment`
- `role` must be one of: `user`, `assistant`, `system` (or NULL)
- `content` size must not exceed 32KB

**Foreign Key Behavior**:
- `ON DELETE CASCADE` - Deleting a thread removes all its items

## State Transitions

### Thread States

```
┌──────────┐    create     ┌──────────┐   add message   ┌──────────┐
│   NEW    │ ───────────▶ │  ACTIVE  │ ───────────────▶│  ACTIVE  │
└──────────┘               └──────────┘                 └──────────┘
                                │                             │
                                │ delete                      │ delete
                                ▼                             ▼
                           ┌──────────┐                  ┌──────────┐
                           │ DELETED  │                  │ DELETED  │
                           └──────────┘                  └──────────┘
```

- **NEW**: Thread created, no messages yet
- **ACTIVE**: Thread has messages, can receive more
- **DELETED**: Thread and all items permanently removed (no soft delete)

### Message Save States (Frontend)

```
┌──────────┐    send      ┌──────────┐   DB success   ┌──────────┐
│  DRAFT   │ ──────────▶ │ PENDING  │ ─────────────▶│  SAVED   │
└──────────┘              └──────────┘                └──────────┘
                               │
                               │ DB failure
                               ▼
                          ┌──────────┐   retry success
                          │  QUEUED  │ ─────────────────┐
                          └──────────┘                  │
                               │                        │
                               │ retry                  │
                               ▼                        ▼
                          ┌──────────┐           ┌──────────┐
                          │ PENDING  │ ─────────▶│  SAVED   │
                          └──────────┘           └──────────┘
```

- **DRAFT**: Message being composed
- **PENDING**: Message sent, awaiting DB confirmation
- **QUEUED**: DB unavailable, stored in local queue
- **SAVED**: Successfully persisted to database

## Pydantic Models (Python)

```python
from datetime import datetime
from typing import Literal, Optional
from pydantic import BaseModel, Field

class ChatThreadMetadata(BaseModel):
    """Thread metadata matching ChatKit ThreadMetadata."""
    id: str = Field(..., pattern=r"^thread_[a-z0-9]+$")
    user_id: Optional[str] = None
    title: Optional[str] = None
    metadata: dict = Field(default_factory=dict)
    created_at: datetime
    updated_at: datetime

class ChatThreadItem(BaseModel):
    """Thread item matching ChatKit ThreadItem."""
    id: str
    thread_id: str
    type: Literal["message", "tool_call", "task", "workflow", "attachment"]
    role: Optional[Literal["user", "assistant", "system"]] = None
    content: dict  # JSON serialized content, max 32KB
    created_at: datetime
    n_tokens: Optional[int] = None

class ChatPage(BaseModel):
    """Paginated response for threads or items."""
    data: list
    has_more: bool
    after: Optional[str] = None
```

## SQL Migration Script

```sql
-- Migration: 005_create_chat_tables.sql

-- Create chat_threads table
CREATE TABLE IF NOT EXISTS chat_threads (
    id VARCHAR(255) PRIMARY KEY,
    user_id VARCHAR(255) REFERENCES users(id) ON DELETE SET NULL,
    title VARCHAR(255),
    metadata JSONB NOT NULL DEFAULT '{}',
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Create chat_thread_items table
CREATE TABLE IF NOT EXISTS chat_thread_items (
    id VARCHAR(255) PRIMARY KEY,
    thread_id VARCHAR(255) NOT NULL REFERENCES chat_threads(id) ON DELETE CASCADE,
    type VARCHAR(50) NOT NULL CHECK (type IN ('message', 'tool_call', 'task', 'workflow', 'attachment')),
    role VARCHAR(50) CHECK (role IN ('user', 'assistant', 'system') OR role IS NULL),
    content JSONB NOT NULL,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    n_tokens INTEGER,
    CONSTRAINT content_size_limit CHECK (octet_length(content::text) <= 32768)
);

-- Indexes for chat_threads
CREATE INDEX IF NOT EXISTS idx_chat_threads_user_id ON chat_threads(user_id);
CREATE INDEX IF NOT EXISTS idx_chat_threads_created_at ON chat_threads(created_at DESC);

-- Indexes for chat_thread_items
CREATE INDEX IF NOT EXISTS idx_chat_thread_items_thread_id ON chat_thread_items(thread_id);
CREATE INDEX IF NOT EXISTS idx_chat_thread_items_thread_created ON chat_thread_items(thread_id, created_at);

-- Update trigger for updated_at
CREATE OR REPLACE FUNCTION update_chat_thread_timestamp()
RETURNS TRIGGER AS $$
BEGIN
    UPDATE chat_threads SET updated_at = NOW() WHERE id = NEW.thread_id;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER chat_item_update_thread_timestamp
AFTER INSERT ON chat_thread_items
FOR EACH ROW EXECUTE FUNCTION update_chat_thread_timestamp();
```

## Data Volume Estimates

| Entity | Estimated Records | Growth Rate |
|--------|-------------------|-------------|
| chat_threads | ~1,000 | ~50/week |
| chat_thread_items | ~100,000 | ~5,000/week |

**Storage Estimate**: ~50MB for 100K messages at average 500 bytes each
