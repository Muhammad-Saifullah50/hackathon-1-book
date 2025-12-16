-- Migration: 004_create_translation_tables.sql
-- Feature: 012-urdu-translation
-- Date: 2025-12-15
-- Description: Creates tables for Urdu translation feature

-- Table: translation_history
-- Tracks which pages users/IPs have translated (metadata only, no content)
CREATE TABLE IF NOT EXISTS translation_history (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id VARCHAR(32),
    ip_hash VARCHAR(32),
    page_url VARCHAR(500) NOT NULL,
    target_language VARCHAR(10) NOT NULL DEFAULT 'ur',
    original_content_hash VARCHAR(64) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),

    -- Foreign key to Better Auth user table (nullable for anonymous users)
    CONSTRAINT fk_th_user_id FOREIGN KEY (user_id) REFERENCES "user"(id) ON DELETE CASCADE,

    -- Either user_id or ip_hash must be set (but not both)
    CONSTRAINT translation_history_user_or_ip CHECK (
        (user_id IS NOT NULL AND ip_hash IS NULL) OR
        (user_id IS NULL AND ip_hash IS NOT NULL)
    ),

    -- Unique per user + page + language (for authenticated users)
    CONSTRAINT translation_history_unique_user UNIQUE (user_id, page_url, target_language),
    -- Unique per ip_hash + page + language (for anonymous users)
    CONSTRAINT translation_history_unique_ip UNIQUE (ip_hash, page_url, target_language)
);

-- Indexes for efficient lookups
CREATE INDEX IF NOT EXISTS idx_th_user_id ON translation_history(user_id) WHERE user_id IS NOT NULL;
CREATE INDEX IF NOT EXISTS idx_th_ip_hash ON translation_history(ip_hash) WHERE ip_hash IS NOT NULL;
CREATE INDEX IF NOT EXISTS idx_th_page_url ON translation_history(page_url);
CREATE INDEX IF NOT EXISTS idx_th_created_at ON translation_history(created_at);

-- Table: translation_quota
-- Tracks daily translation usage per user or IP for rate limiting
CREATE TABLE IF NOT EXISTS translation_quota (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id VARCHAR(32),
    ip_hash VARCHAR(32),
    quota_date DATE NOT NULL DEFAULT CURRENT_DATE,
    used_count INTEGER NOT NULL DEFAULT 0 CHECK (used_count >= 0),
    daily_limit INTEGER NOT NULL DEFAULT 5,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),

    -- Foreign key to Better Auth user table (nullable for anonymous users)
    CONSTRAINT fk_tq_user_id FOREIGN KEY (user_id) REFERENCES "user"(id) ON DELETE CASCADE,

    -- Either user_id or ip_hash must be set (but not both)
    CONSTRAINT translation_quota_user_or_ip CHECK (
        (user_id IS NOT NULL AND ip_hash IS NULL) OR
        (user_id IS NULL AND ip_hash IS NOT NULL)
    ),

    -- Unique per user + date (for authenticated users)
    CONSTRAINT translation_quota_unique_user UNIQUE (user_id, quota_date),
    -- Unique per ip_hash + date (for anonymous users)
    CONSTRAINT translation_quota_unique_ip UNIQUE (ip_hash, quota_date)
);

-- Indexes for fast quota lookup
CREATE INDEX IF NOT EXISTS idx_tq_user_id ON translation_quota(user_id) WHERE user_id IS NOT NULL;
CREATE INDEX IF NOT EXISTS idx_tq_ip_hash ON translation_quota(ip_hash) WHERE ip_hash IS NOT NULL;
CREATE INDEX IF NOT EXISTS idx_tq_date ON translation_quota(quota_date);

-- Trigger for translation_history updated_at
-- Note: Reuses update_updated_at_column() function from personalization migration
DROP TRIGGER IF EXISTS trg_th_updated_at ON translation_history;
CREATE TRIGGER trg_th_updated_at
    BEFORE UPDATE ON translation_history
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

-- Trigger for translation_quota updated_at
DROP TRIGGER IF EXISTS trg_tq_updated_at ON translation_quota;
CREATE TRIGGER trg_tq_updated_at
    BEFORE UPDATE ON translation_quota
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

-- Comments for documentation
COMMENT ON TABLE translation_history IS 'Tracks which pages users/IPs have translated (metadata only, no content)';
COMMENT ON TABLE translation_quota IS 'Daily translation quota tracking per user or IP address';
COMMENT ON COLUMN translation_history.ip_hash IS 'SHA-256 hash (truncated to 32 chars) of client IP for anonymous users';
COMMENT ON COLUMN translation_history.original_content_hash IS 'SHA-256 hash of original page content (detect source changes)';
COMMENT ON COLUMN translation_quota.used_count IS 'Number of translations used today (max 5)';
COMMENT ON COLUMN translation_quota.daily_limit IS 'Maximum translations per day (default 5)';
