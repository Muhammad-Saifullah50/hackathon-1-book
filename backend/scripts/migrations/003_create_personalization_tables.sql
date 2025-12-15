-- Migration: 003_create_personalization_tables.sql
-- Feature: 011-personalize-page
-- Date: 2025-12-14
-- Description: Creates tables for page personalization feature

-- Table: personalization_history
-- Stores lightweight metadata about personalized pages (NO content storage)
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
-- Tracks daily personalization usage per user for rate limiting
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
