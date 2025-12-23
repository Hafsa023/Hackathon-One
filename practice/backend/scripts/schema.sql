-- Physical AI Book Chatbot Database Schema
-- For Neon Serverless Postgres

-- Chat History Table
-- Stores all question-answer pairs for session tracking
CREATE TABLE IF NOT EXISTS chat_history (
    id SERIAL PRIMARY KEY,
    session_id VARCHAR(64) NOT NULL,
    question TEXT NOT NULL,
    answer TEXT NOT NULL,
    chapter VARCHAR(255),
    sources JSONB DEFAULT '[]',
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

-- Indexes for efficient queries
CREATE INDEX IF NOT EXISTS idx_chat_history_session_id ON chat_history(session_id);
CREATE INDEX IF NOT EXISTS idx_chat_history_created_at ON chat_history(created_at);
CREATE INDEX IF NOT EXISTS idx_chat_history_chapter ON chat_history(chapter);

-- Analytics Events Table
-- Tracks usage patterns and user behavior
CREATE TABLE IF NOT EXISTS analytics_events (
    id SERIAL PRIMARY KEY,
    event_type VARCHAR(50) NOT NULL,
    session_id VARCHAR(64),
    chapter VARCHAR(255),
    metadata JSONB DEFAULT '{}',
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

-- Indexes for analytics queries
CREATE INDEX IF NOT EXISTS idx_analytics_event_type ON analytics_events(event_type);
CREATE INDEX IF NOT EXISTS idx_analytics_session_id ON analytics_events(session_id);
CREATE INDEX IF NOT EXISTS idx_analytics_created_at ON analytics_events(created_at);

-- Useful Analytics Queries
-- ============================

-- Most asked questions by chapter
-- SELECT chapter, COUNT(*) as question_count
-- FROM chat_history
-- WHERE chapter IS NOT NULL
-- GROUP BY chapter
-- ORDER BY question_count DESC;

-- Daily usage statistics
-- SELECT DATE(created_at) as date, COUNT(*) as questions
-- FROM chat_history
-- GROUP BY DATE(created_at)
-- ORDER BY date DESC;

-- Average questions per session
-- SELECT AVG(question_count) as avg_questions_per_session
-- FROM (
--     SELECT session_id, COUNT(*) as question_count
--     FROM chat_history
--     GROUP BY session_id
-- ) session_counts;

-- Selected text usage
-- SELECT
--     metadata->>'has_selected_text' as used_selection,
--     COUNT(*) as count
-- FROM analytics_events
-- WHERE event_type = 'chat_question'
-- GROUP BY metadata->>'has_selected_text';
