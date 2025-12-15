# Data Model: Page Content Personalization

**Feature**: 011-personalize-page
**Date**: 2025-12-14
**Database**: Neon PostgreSQL

## Entity Relationship Diagram

```
┌─────────────────────┐       ┌─────────────────────────┐
│       user          │       │  personalization_history │
│  (Better Auth)      │       │                         │
├─────────────────────┤       ├─────────────────────────┤
│ id (PK)             │──────<│ user_id (FK)            │
│ email               │       │ page_url                │
│ name                │       │ profile_hash            │
│ ...                 │       │ created_at              │
└─────────────────────┘       │ updated_at              │
         │                    └─────────────────────────┘
         │
         │                    ┌─────────────────────────┐
         │                    │  personalization_quota   │
         │                    ├─────────────────────────┤
         └───────────────────<│ user_id (FK)            │
                              │ date                    │
                              │ used_count              │
                              │ created_at              │
                              │ updated_at              │
                              └─────────────────────────┘
```

## Database Tables

### 1. personalization_history

Stores lightweight metadata about personalized pages (NO content storage).

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | SERIAL | PRIMARY KEY | Auto-incrementing ID |
| user_id | VARCHAR(32) | NOT NULL, FK → user.id | Better Auth NanoID |
| page_url | VARCHAR(500) | NOT NULL | URL path (e.g., "/docs/module-1/intro") |
| profile_hash | VARCHAR(64) | NOT NULL | SHA-256 hash of profile at personalization time |
| original_content_hash | VARCHAR(64) | NULL | Hash of original content (detect source changes) |
| created_at | TIMESTAMP | DEFAULT NOW() | First personalization timestamp |
| updated_at | TIMESTAMP | DEFAULT NOW() | Last personalization timestamp |

**Constraints:**
- `UNIQUE(user_id, page_url)` - One record per user per page
- `INDEX idx_ph_user_id ON personalization_history(user_id)` - Fast user history lookup

**Storage Estimate:**
- ~150 bytes per record (well under 200-byte requirement)
- 1000 users × 20 pages each = 20,000 records ≈ 3 MB

### 2. personalization_quota

Tracks daily personalization usage per user for rate limiting.

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | SERIAL | PRIMARY KEY | Auto-incrementing ID |
| user_id | VARCHAR(32) | NOT NULL, FK → user.id | Better Auth NanoID |
| date | DATE | NOT NULL | UTC date for quota tracking |
| used_count | INTEGER | DEFAULT 0, CHECK >= 0 | Personalizations used today |
| created_at | TIMESTAMP | DEFAULT NOW() | Record creation timestamp |
| updated_at | TIMESTAMP | DEFAULT NOW() | Last update timestamp |

**Constraints:**
- `UNIQUE(user_id, date)` - One quota record per user per day
- `INDEX idx_pq_user_date ON personalization_quota(user_id, date)` - Fast quota lookup

**Business Rules:**
- Daily limit: 5 personalizations per user
- Reset: Midnight UTC (new date = new quota record)
- Free re-personalization: Doesn't increment `used_count` for existing history entries

## SQL Migration Script

```sql
-- Migration: 003_create_personalization_tables.sql
-- Feature: 011-personalize-page
-- Date: 2025-12-14

-- Table: personalization_history
CREATE TABLE IF NOT EXISTS personalization_history (
    id SERIAL PRIMARY KEY,
    user_id VARCHAR(32) NOT NULL,
    page_url VARCHAR(500) NOT NULL,
    profile_hash VARCHAR(64) NOT NULL,
    original_content_hash VARCHAR(64),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),

    -- Foreign key to Better Auth user table
    CONSTRAINT fk_ph_user_id FOREIGN KEY (user_id) REFERENCES "user"(id) ON DELETE CASCADE,

    -- One record per user per page
    CONSTRAINT uq_ph_user_page UNIQUE (user_id, page_url)
);

-- Index for fast user history lookup
CREATE INDEX IF NOT EXISTS idx_ph_user_id ON personalization_history(user_id);

-- Index for profile hash queries (detect stale personalizations)
CREATE INDEX IF NOT EXISTS idx_ph_profile_hash ON personalization_history(user_id, profile_hash);

-- Table: personalization_quota
CREATE TABLE IF NOT EXISTS personalization_quota (
    id SERIAL PRIMARY KEY,
    user_id VARCHAR(32) NOT NULL,
    date DATE NOT NULL,
    used_count INTEGER DEFAULT 0 CHECK (used_count >= 0),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),

    -- Foreign key to Better Auth user table
    CONSTRAINT fk_pq_user_id FOREIGN KEY (user_id) REFERENCES "user"(id) ON DELETE CASCADE,

    -- One quota record per user per day
    CONSTRAINT uq_pq_user_date UNIQUE (user_id, date)
);

-- Index for fast quota lookup
CREATE INDEX IF NOT EXISTS idx_pq_user_date ON personalization_quota(user_id, date);

-- Function to update updated_at timestamp
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

-- Trigger for personalization_history
DROP TRIGGER IF EXISTS trg_ph_updated_at ON personalization_history;
CREATE TRIGGER trg_ph_updated_at
    BEFORE UPDATE ON personalization_history
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

-- Trigger for personalization_quota
DROP TRIGGER IF EXISTS trg_pq_updated_at ON personalization_quota;
CREATE TRIGGER trg_pq_updated_at
    BEFORE UPDATE ON personalization_quota
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

-- Comments for documentation
COMMENT ON TABLE personalization_history IS 'Tracks which pages users have personalized (metadata only, no content)';
COMMENT ON TABLE personalization_quota IS 'Daily personalization quota tracking per user';
COMMENT ON COLUMN personalization_history.profile_hash IS 'SHA-256 hash of user profile at personalization time';
COMMENT ON COLUMN personalization_history.original_content_hash IS 'SHA-256 hash of original page content (detect source changes)';
COMMENT ON COLUMN personalization_quota.used_count IS 'Number of personalizations used today (max 5)';
```

## Pydantic Models (Backend)

```python
# backend/src/models/personalization.py

from datetime import date, datetime
from typing import Optional
from pydantic import BaseModel, Field

class PersonalizationHistoryCreate(BaseModel):
    """Create a new personalization history record."""
    user_id: str
    page_url: str = Field(max_length=500)
    profile_hash: str = Field(max_length=64)
    original_content_hash: Optional[str] = Field(None, max_length=64)

class PersonalizationHistoryRecord(BaseModel):
    """Personalization history record from database."""
    id: int
    user_id: str
    page_url: str
    profile_hash: str
    original_content_hash: Optional[str]
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class PersonalizationQuotaRecord(BaseModel):
    """Daily quota record from database."""
    id: int
    user_id: str
    date: date
    used_count: int
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class QuotaStatus(BaseModel):
    """Quota status response."""
    limit: int = 5
    used: int
    remaining: int
    resets_at: datetime  # Midnight UTC next day

class PersonalizationRequest(BaseModel):
    """Request to personalize page content."""
    page_url: str = Field(max_length=500)
    page_content: str = Field(min_length=1)
    is_free_repersonalization: bool = False

class PersonalizationResponse(BaseModel):
    """Response with personalized content."""
    personalized_content: str
    profile_hash: str
    original_content_hash: str
    processing_time_ms: int
    quota_remaining: int

class PersonalizationHistoryResponse(BaseModel):
    """User's personalization history."""
    pages: list[dict]  # [{url, profile_hash, created_at}]
    current_profile_hash: str
```

## TypeScript Types (Frontend)

```typescript
// website/src/types/personalization.ts

export interface PersonalizedPageCache {
  content: string;
  profileHash: string;
  originalContentHash: string;
  timestamp: string;
  pageUrl: string;
  lastAccessed: string;
}

export interface QuotaStatus {
  limit: number;
  used: number;
  remaining: number;
  resetsAt: string;  // ISO datetime
}

export interface PersonalizationRequest {
  pageUrl: string;
  pageContent: string;
  isFreeRepersonalization: boolean;
}

export interface PersonalizationResponse {
  personalizedContent: string;
  profileHash: string;
  originalContentHash: string;
  processingTimeMs: number;
  quotaRemaining: number;
}

export interface PersonalizationHistoryItem {
  url: string;
  profileHash: string;
  createdAt: string;
}

export interface PersonalizationHistoryResponse {
  pages: PersonalizationHistoryItem[];
  currentProfileHash: string;
}
```

## State Transitions

### Personalization History States

```
┌─────────────┐     personalize()     ┌─────────────┐
│  NO_RECORD  │ ───────────────────> │   ACTIVE    │
└─────────────┘                       └─────────────┘
                                            │
                                            │ profile changed
                                            ▼
                                      ┌─────────────┐
                                      │    STALE    │
                                      │ (hash diff) │
                                      └─────────────┘
                                            │
                                            │ re-personalize (free)
                                            ▼
                                      ┌─────────────┐
                                      │   ACTIVE    │
                                      │ (new hash)  │
                                      └─────────────┘
```

### Quota States

```
┌─────────────┐     first request     ┌─────────────┐
│  NO_QUOTA   │ ───────────────────> │ AVAILABLE   │
│  (new day)  │                       │ (used < 5)  │
└─────────────┘                       └─────────────┘
                                            │
                                            │ personalize (used++)
                                            ▼
                                      ┌─────────────┐
                                      │  EXHAUSTED  │
                                      │ (used = 5)  │
                                      └─────────────┘
                                            │
                                            │ midnight UTC
                                            ▼
                                      ┌─────────────┐
                                      │  AVAILABLE  │
                                      │ (new date)  │
                                      └─────────────┘
```

## Validation Rules

| Entity | Field | Rule |
|--------|-------|------|
| PersonalizationHistory | page_url | Max 500 chars, valid URL path |
| PersonalizationHistory | profile_hash | Exactly 64 hex chars (SHA-256) |
| PersonalizationHistory | user_id | Valid Better Auth NanoID |
| PersonalizationQuota | used_count | >= 0, checked by database |
| PersonalizationQuota | date | UTC date only, no time component |

## Query Patterns

```sql
-- Get user's personalization history
SELECT page_url, profile_hash, created_at
FROM personalization_history
WHERE user_id = $1
ORDER BY updated_at DESC;

-- Check if page was personalized (cross-device check)
SELECT profile_hash, original_content_hash
FROM personalization_history
WHERE user_id = $1 AND page_url = $2;

-- Get or create today's quota
INSERT INTO personalization_quota (user_id, date, used_count)
VALUES ($1, CURRENT_DATE, 0)
ON CONFLICT (user_id, date) DO NOTHING
RETURNING *;

-- Increment quota (returns new count)
UPDATE personalization_quota
SET used_count = used_count + 1
WHERE user_id = $1 AND date = CURRENT_DATE AND used_count < 5
RETURNING used_count;

-- Upsert personalization history
INSERT INTO personalization_history (user_id, page_url, profile_hash, original_content_hash)
VALUES ($1, $2, $3, $4)
ON CONFLICT (user_id, page_url)
DO UPDATE SET
    profile_hash = EXCLUDED.profile_hash,
    original_content_hash = EXCLUDED.original_content_hash,
    updated_at = NOW();
```
