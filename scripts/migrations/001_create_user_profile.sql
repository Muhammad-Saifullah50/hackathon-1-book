-- Migration: Create user_profile table for learner profiles
-- Date: 2025-12-13
-- Feature: 010-neon-database-migration
--
-- This migration creates the user_profile table that stores learner preferences
-- and personalization data. The user_id column references Better Auth's user table.
--
-- Prerequisites:
-- - Better Auth schema must be created first (user, account, session tables)
-- - Run: npx @better-auth/cli generate && npx @better-auth/cli migrate

-- Create user_profile table
CREATE TABLE IF NOT EXISTS user_profile (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id TEXT NOT NULL UNIQUE,
    age_range VARCHAR(20),
    education_level VARCHAR(20),
    tech_background VARCHAR(30),
    primary_goal VARCHAR(30),
    learning_mode VARCHAR(20),
    learning_speed VARCHAR(20),
    time_per_week INTEGER CHECK (time_per_week >= 0),
    preferred_language VARCHAR(10) NOT NULL DEFAULT 'en',
    created_at TIMESTAMP NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP NOT NULL DEFAULT NOW(),

    -- Foreign key to Better Auth user table
    CONSTRAINT fk_user_profile_user
        FOREIGN KEY (user_id)
        REFERENCES "user"(id)
        ON DELETE CASCADE
);

-- Create index for faster lookups by user_id
CREATE INDEX IF NOT EXISTS idx_user_profile_user_id ON user_profile(user_id);

-- Add check constraints for enum-like fields
ALTER TABLE user_profile
    ADD CONSTRAINT chk_age_range
        CHECK (age_range IS NULL OR age_range IN ('under_18', '18_24', '25_34', '35_44', '45_plus'));

ALTER TABLE user_profile
    ADD CONSTRAINT chk_education_level
        CHECK (education_level IS NULL OR education_level IN ('high_school', 'bachelors', 'masters', 'phd', 'self_taught'));

ALTER TABLE user_profile
    ADD CONSTRAINT chk_tech_background
        CHECK (tech_background IS NULL OR tech_background IN ('none', 'beginner', 'intermediate', 'advanced'));

ALTER TABLE user_profile
    ADD CONSTRAINT chk_primary_goal
        CHECK (primary_goal IS NULL OR primary_goal IN ('career', 'research', 'hobby', 'education'));

ALTER TABLE user_profile
    ADD CONSTRAINT chk_learning_mode
        CHECK (learning_mode IS NULL OR learning_mode IN ('visual', 'reading', 'hands_on', 'mixed'));

ALTER TABLE user_profile
    ADD CONSTRAINT chk_learning_speed
        CHECK (learning_speed IS NULL OR learning_speed IN ('thorough', 'balanced', 'accelerated'));

-- Create trigger to auto-update updated_at timestamp
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ language 'plpgsql';

CREATE TRIGGER update_user_profile_updated_at
    BEFORE UPDATE ON user_profile
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

-- Comment on table
COMMENT ON TABLE user_profile IS 'Learner profile data for personalized learning experience';
COMMENT ON COLUMN user_profile.user_id IS 'FK to Better Auth user.id (1:1 relationship)';
COMMENT ON COLUMN user_profile.age_range IS 'Learner age bracket for content adaptation';
COMMENT ON COLUMN user_profile.education_level IS 'Highest education level achieved';
COMMENT ON COLUMN user_profile.tech_background IS 'Technical experience level';
COMMENT ON COLUMN user_profile.primary_goal IS 'Main motivation for learning';
COMMENT ON COLUMN user_profile.learning_mode IS 'Preferred learning style';
COMMENT ON COLUMN user_profile.learning_speed IS 'Desired pace of learning';
COMMENT ON COLUMN user_profile.time_per_week IS 'Hours available per week for learning';
COMMENT ON COLUMN user_profile.preferred_language IS 'Content language preference (ISO 639-1 code)';
