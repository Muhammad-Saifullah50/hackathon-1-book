-- Migration: Add focus_area column to user_profile table
-- Date: 2025-12-14
-- Feature: 010-neon-database-migration
--
-- This migration adds the focus_area column to store learner preference
-- for hardware-focused or software-focused content.
--
-- Values:
--   - 'hardware': Robotics, sensors, actuators, mechanical systems
--   - 'software': AI, algorithms, simulation, programming

-- Add focus_area column
ALTER TABLE user_profile
    ADD COLUMN IF NOT EXISTS focus_area VARCHAR(20);

-- Add check constraint for enum validation
ALTER TABLE user_profile
    ADD CONSTRAINT chk_focus_area
        CHECK (focus_area IS NULL OR focus_area IN ('hardware', 'software'));

-- Add comment for documentation
COMMENT ON COLUMN user_profile.focus_area IS 'Learner focus preference: hardware (robotics, sensors) or software (AI, algorithms)';
