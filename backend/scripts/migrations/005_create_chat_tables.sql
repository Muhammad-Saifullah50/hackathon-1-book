-- Migration: 005_create_chat_tables.sql
-- Description: Create tables for chat thread persistence with Neon PostgreSQL
-- Feature: 013-chatbot-persistence
-- Date: 2025-12-16

-- Create chat_threads table
CREATE TABLE IF NOT EXISTS chat_threads (
    id VARCHAR(255) PRIMARY KEY,
    user_id VARCHAR(255),
    title VARCHAR(255),
    metadata JSONB NOT NULL DEFAULT '{}',
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),

    -- Foreign key to Better Auth user table (nullable for anonymous sessions)
    CONSTRAINT fk_ct_user_id FOREIGN KEY (user_id) REFERENCES "user"(id) ON DELETE SET NULL
);

-- Create chat_thread_items table
CREATE TABLE IF NOT EXISTS chat_thread_items (
    id VARCHAR(255) PRIMARY KEY,
    thread_id VARCHAR(255) NOT NULL REFERENCES chat_threads(id) ON DELETE CASCADE,
    type VARCHAR(50) NOT NULL CHECK (type IN (
        'user_message',
        'assistant_message',
        'client_tool_call',
        'sdk_tool_call',
        'sdk_hidden_context',
        'end_of_turn',
        'message',
        'tool_call',
        'task',
        'workflow',
        'attachment'
    )),
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

-- Update trigger for updated_at on chat_threads
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

-- Comments
COMMENT ON TABLE chat_threads IS 'Chat conversation threads for RAG tutor chatbot';
COMMENT ON TABLE chat_thread_items IS 'Individual messages and items within chat threads';
COMMENT ON COLUMN chat_threads.metadata IS 'Arbitrary JSON metadata (e.g., previous_response_id for Agent SDK)';
COMMENT ON COLUMN chat_thread_items.content IS 'Full item content serialized as JSON, max 32KB';
