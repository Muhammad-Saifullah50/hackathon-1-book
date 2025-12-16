# Data Model: Urdu Translation Feature

**Feature**: 012-urdu-translation
**Date**: 2025-12-15

## Overview

This document defines the data model for the Urdu translation feature, including database tables, API request/response models, and client-side cache schema.

## Database Schema (PostgreSQL/Neon)

### Table: `translation_history`

Tracks which pages users have translated for cross-device awareness.

```sql
CREATE TABLE translation_history (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES user_profiles(id) ON DELETE CASCADE,
    ip_hash VARCHAR(32),  -- For anonymous users (SHA-256 truncated)
    page_url VARCHAR(500) NOT NULL,
    target_language VARCHAR(10) NOT NULL DEFAULT 'ur',
    original_content_hash VARCHAR(64) NOT NULL,  -- SHA-256 of original content
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),

    -- Either user_id or ip_hash must be set
    CONSTRAINT translation_history_user_or_ip CHECK (
        (user_id IS NOT NULL AND ip_hash IS NULL) OR
        (user_id IS NULL AND ip_hash IS NOT NULL)
    ),

    -- Unique per user/ip + page + language
    CONSTRAINT translation_history_unique_user UNIQUE (user_id, page_url, target_language),
    CONSTRAINT translation_history_unique_ip UNIQUE (ip_hash, page_url, target_language)
);

-- Indexes for efficient lookups
CREATE INDEX idx_translation_history_user_id ON translation_history(user_id);
CREATE INDEX idx_translation_history_ip_hash ON translation_history(ip_hash);
CREATE INDEX idx_translation_history_page_url ON translation_history(page_url);
CREATE INDEX idx_translation_history_created_at ON translation_history(created_at);
```

### Table: `translation_quota`

Tracks daily translation usage per user or IP for quota enforcement.

```sql
CREATE TABLE translation_quota (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES user_profiles(id) ON DELETE CASCADE,
    ip_hash VARCHAR(32),  -- For anonymous users
    quota_date DATE NOT NULL DEFAULT CURRENT_DATE,
    used_count INTEGER NOT NULL DEFAULT 0,
    daily_limit INTEGER NOT NULL DEFAULT 5,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),

    -- Either user_id or ip_hash must be set
    CONSTRAINT translation_quota_user_or_ip CHECK (
        (user_id IS NOT NULL AND ip_hash IS NULL) OR
        (user_id IS NULL AND ip_hash IS NOT NULL)
    ),

    -- Unique per user/ip + date
    CONSTRAINT translation_quota_unique_user UNIQUE (user_id, quota_date),
    CONSTRAINT translation_quota_unique_ip UNIQUE (ip_hash, quota_date)
);

-- Indexes
CREATE INDEX idx_translation_quota_user_id ON translation_quota(user_id);
CREATE INDEX idx_translation_quota_ip_hash ON translation_quota(ip_hash);
CREATE INDEX idx_translation_quota_date ON translation_quota(quota_date);

-- Cleanup old quota records (> 7 days) - run via cron job
-- DELETE FROM translation_quota WHERE quota_date < CURRENT_DATE - INTERVAL '7 days';
```

## Pydantic Models (Backend)

### Request Models

```python
from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime

class TranslationRequest(BaseModel):
    """Request to translate page content to Urdu."""
    page_content: str = Field(..., min_length=1, max_length=100000, description="Markdown content to translate")
    page_url: str = Field(..., min_length=1, max_length=500, description="URL of the page being translated")
    source_language: str = Field(default="en", description="Source language code")
    target_language: str = Field(default="ur", description="Target language code (Urdu)")

class TranslationHistoryCheckRequest(BaseModel):
    """Request to check if a page was previously translated."""
    page_url: str = Field(..., min_length=1, max_length=500)
    target_language: str = Field(default="ur")
```

### Response Models

```python
class TranslationResponse(BaseModel):
    """Response containing translated content."""
    translated_content: str = Field(..., description="Translated markdown content")
    source_language: str = Field(default="en")
    target_language: str = Field(default="ur")
    processing_time_ms: int = Field(..., description="Time taken to translate in milliseconds")
    original_content_hash: str = Field(..., description="SHA-256 hash of original content")
    quota_remaining: int = Field(..., description="Remaining translations for today")
    quota_limit: int = Field(default=5, description="Daily translation limit")

class QuotaStatusResponse(BaseModel):
    """Current quota status for the user/IP."""
    used: int = Field(..., description="Translations used today")
    remaining: int = Field(..., description="Translations remaining today")
    limit: int = Field(default=5, description="Daily limit")
    resets_at: datetime = Field(..., description="When quota resets (midnight UTC)")

class TranslationHistoryResponse(BaseModel):
    """Response for translation history check."""
    has_translation: bool = Field(..., description="Whether page was previously translated")
    original_content_hash: Optional[str] = Field(None, description="Hash of original content at translation time")
    translated_at: Optional[datetime] = Field(None, description="When the page was translated")
    content_changed: bool = Field(default=False, description="Whether original content has changed since translation")

class ErrorResponse(BaseModel):
    """Standard error response."""
    error: str = Field(..., description="Error message")
    code: str = Field(..., description="Error code")
    details: Optional[dict] = Field(None, description="Additional error details")
```

## TypeScript Interfaces (Frontend)

### API Types

```typescript
// types/translation.ts

export interface TranslationRequest {
  page_content: string;
  page_url: string;
  source_language?: string; // default: "en"
  target_language?: string; // default: "ur"
}

export interface TranslationResponse {
  translated_content: string;
  source_language: string;
  target_language: string;
  processing_time_ms: number;
  original_content_hash: string;
  quota_remaining: number;
  quota_limit: number;
}

export interface QuotaStatus {
  used: number;
  remaining: number;
  limit: number;
  resets_at: string; // ISO datetime
}

export interface TranslationHistoryCheck {
  has_translation: boolean;
  original_content_hash?: string;
  translated_at?: string; // ISO datetime
  content_changed: boolean;
}
```

### Cache Types (localStorage)

```typescript
export interface TranslatedPageCache {
  content: string;              // Translated markdown
  originalContentHash: string;  // SHA-256 of original content
  timestamp: string;            // ISO date when translated
  pageUrl: string;              // Page URL for validation
  targetLanguage: string;       // Always "ur" for now
}

// localStorage key format: `translated_${targetLanguage}_${encodeURIComponent(pageUrl)}`
// Example: `translated_ur_/docs/module-01/embodied-intelligence`
```

### Component State Types

```typescript
export type TranslationState =
  | 'idle'           // Initial state, no translation
  | 'loading'        // Translation in progress
  | 'translated'     // Viewing translated content
  | 'error'          // Translation failed
  | 'quota_exceeded' // Daily limit reached

export type ViewMode = 'original' | 'translated'

export interface UseTranslationReturn {
  state: TranslationState;
  viewMode: ViewMode;
  error: string | null;
  quotaStatus: QuotaStatus | null;
  translatedContent: string | null;
  hasCachedVersion: boolean;
  isContentStale: boolean;  // Original content changed since translation
  translate: (content: string, pageUrl: string) => Promise<void>;
  toggleView: () => void;
  loadCachedVersion: () => void;
}
```

## Entity Relationships

```
┌──────────────────┐     ┌───────────────────────┐
│  user_profiles   │     │  translation_history  │
│  (existing)      │────<│                       │
│                  │     │  - user_id (FK)       │
│  - id (PK)       │     │  - ip_hash            │
│  - email         │     │  - page_url           │
│  - ...           │     │  - target_language    │
└──────────────────┘     │  - original_hash      │
                         │  - created_at         │
                         └───────────────────────┘

┌──────────────────┐     ┌───────────────────────┐
│  user_profiles   │     │  translation_quota    │
│  (existing)      │────<│                       │
│                  │     │  - user_id (FK)       │
│                  │     │  - ip_hash            │
│                  │     │  - quota_date         │
│                  │     │  - used_count         │
│                  │     │  - daily_limit        │
└──────────────────┘     └───────────────────────┘

Client-Side (localStorage):
┌────────────────────────────────────────┐
│  TranslatedPageCache                   │
│  Key: translated_ur_{pageUrl}          │
│  - content: string                     │
│  - originalContentHash: string         │
│  - timestamp: ISO date                 │
│  - pageUrl: string                     │
│  - targetLanguage: "ur"                │
└────────────────────────────────────────┘
```

## Validation Rules

| Field | Rule |
|-------|------|
| `page_content` | 1-100,000 characters |
| `page_url` | 1-500 characters, valid URL path |
| `target_language` | Must be "ur" (Urdu) |
| `used_count` | 0 to daily_limit (default 5) |
| `ip_hash` | Exactly 32 characters (truncated SHA-256) |
| `original_content_hash` | Exactly 64 characters (full SHA-256) |

## State Transitions

### Translation State Machine

```
                    ┌──────────────────────────────┐
                    │                              │
                    ▼                              │
┌────────┐    ┌─────────┐    ┌────────────┐    ┌──────┐
│  idle  │───>│ loading │───>│ translated │───>│ idle │
└────────┘    └─────────┘    └────────────┘    └──────┘
    │              │              │                 ▲
    │              │              │                 │
    │              ▼              ▼                 │
    │         ┌─────────┐    ┌────────────────┐    │
    └────────>│  error  │    │ quota_exceeded │────┘
              └─────────┘    └────────────────┘
                  │                  │
                  │                  │
                  └──────────────────┘
                        (retry)
```

### View Mode Transitions

```
┌──────────┐                    ┌────────────┐
│ original │◄──── toggleView ──►│ translated │
└──────────┘                    └────────────┘
```
