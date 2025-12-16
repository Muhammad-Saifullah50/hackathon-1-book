"""
Integration tests for chat persistence with Neon database.

Tests the full flow of storing and retrieving chat history.
Following TDD Red-Green-Refactor cycle.
"""

import pytest
import asyncpg
import os
import json
from datetime import datetime, timezone
from dotenv import load_dotenv

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))

from src.services.chat_store import NeonChatKitStore
from chatkit.types import ThreadMetadata, ThreadItem

load_dotenv()


@pytest.fixture
async def db_connection():
    """Create a test database connection."""
    conn = await asyncpg.connect(os.getenv("DATABASE_URL"))
    # Create test user if it doesn't exist
    try:
        await conn.execute("""
            INSERT INTO "user" (id, email, name, created_at, updated_at)
            VALUES ('user_test123', 'test@example.com', 'Test User', NOW(), NOW())
            ON CONFLICT (id) DO NOTHING
        """)
    except Exception as e:
        print(f"Warning: Could not create test user: {e}")

    yield conn
    await conn.close()


@pytest.fixture
async def clean_test_data(db_connection):
    """Clean up test data before and after each test."""
    # Clean before
    await db_connection.execute("DELETE FROM chat_thread_items WHERE thread_id LIKE 'thread_test%'")
    await db_connection.execute("DELETE FROM chat_threads WHERE id LIKE 'thread_test%'")

    yield

    # Clean after
    await db_connection.execute("DELETE FROM chat_thread_items WHERE thread_id LIKE 'thread_test%'")
    await db_connection.execute("DELETE FROM chat_threads WHERE id LIKE 'thread_test%'")


@pytest.fixture
def store():
    """Create NeonChatKitStore instance for integration tests."""
    return NeonChatKitStore(os.getenv("DATABASE_URL"))


@pytest.fixture
def mock_context():
    """Create mock context with user ID."""
    return {"user_id": "user_test123"}


# T013: Integration test for loading thread history
@pytest.mark.asyncio
async def test_load_thread_history_flow(store, db_connection, clean_test_data, mock_context):
    """
    Test the complete flow of loading thread history.

    This test will fail until User Story 1 is fully implemented (T014-T016).
    """
    # Setup: Insert a test thread directly into database
    thread_id = "thread_test_integration_001"
    await db_connection.execute(
        """
        INSERT INTO chat_threads (id, user_id, title, metadata, created_at, updated_at)
        VALUES ($1, $2, $3, $4, $5, $6)
        """,
        thread_id,
        "user_test123",
        "Test Thread",
        json.dumps({"test": "data"}),
        datetime.now(timezone.utc),
        datetime.now(timezone.utc)
    )

    # Setup: Insert test messages
    await db_connection.execute(
        """
        INSERT INTO chat_thread_items (id, thread_id, type, role, content, created_at)
        VALUES ($1, $2, $3, $4, $5, $6)
        """,
        "message_test_001",
        thread_id,
        "message",
        "user",
        json.dumps({"text": "Hello"}),
        datetime.now(timezone.utc)
    )

    # Test: Load the thread
    thread = await store.load_thread(thread_id, mock_context)
    assert thread.id == thread_id
    assert thread.title == "Test Thread"

    # Test: Load thread items
    page = await store.load_thread_items(thread_id, None, 50, "asc", mock_context)
    assert len(page.data) == 1
    assert page.data[0].id == "message_test_001"
    assert page.has_more is False


@pytest.mark.asyncio
async def test_load_threads_for_user(store, db_connection, clean_test_data, mock_context):
    """
    Test loading all threads for a specific user.

    This test will fail until T016 is implemented.
    """
    # Setup: Insert multiple threads for the test user
    for i in range(3):
        await db_connection.execute(
            """
            INSERT INTO chat_threads (id, user_id, created_at, updated_at)
            VALUES ($1, $2, $3, $4)
            """,
            f"thread_test_{i:03d}",
            "user_test123",
            datetime.now(timezone.utc),
            datetime.now(timezone.utc)
        )

    # Test: Load threads
    page = await store.load_threads(10, None, "desc", mock_context)

    assert len(page.data) >= 3
    assert all(t.user_id == "user_test123" for t in page.data)


# T023: Integration test for message persistence flow
@pytest.mark.asyncio
async def test_message_persistence_flow(store, db_connection, clean_test_data, mock_context):
    """
    Test the complete flow of persisting messages.

    This test will fail until User Story 2 is fully implemented (T024-T028).
    Tests:
    1. Creating a new thread
    2. Adding messages to the thread
    3. Updating existing messages
    4. Loading messages back
    """
    # Step 1: Create and save a new thread
    thread = ThreadMetadata(
        id="thread_test_persist_001",
        created_at=datetime.now(timezone.utc),
        updated_at=datetime.now(timezone.utc),
        metadata={"topic": "integration test"}
    )

    await store.save_thread(thread, mock_context)

    # Verify thread was saved
    loaded_thread = await store.load_thread("thread_test_persist_001", mock_context)
    assert loaded_thread.id == thread.id
    assert loaded_thread.metadata == {"topic": "integration test"}

    # Step 2: Add a user message
    user_message = ThreadItem.model_validate({
        "id": "message_persist_user_001",
        "type": "message",
        "role": "user",
        "content": {"text": "What is robotics?"},
        "created_at": datetime.now(timezone.utc)
    })

    await store.add_thread_item("thread_test_persist_001", user_message, mock_context)

    # Step 3: Add an assistant response
    assistant_message = ThreadItem.model_validate({
        "id": "message_persist_assistant_001",
        "type": "message",
        "role": "assistant",
        "content": {"text": "Robotics is the study of robots..."},
        "created_at": datetime.now(timezone.utc)
    })

    await store.add_thread_item("thread_test_persist_001", assistant_message, mock_context)

    # Step 4: Load all messages
    page = await store.load_thread_items("thread_test_persist_001", None, 50, "asc", mock_context)

    assert len(page.data) == 2
    assert page.data[0].id == "message_persist_user_001"
    assert page.data[0].content["text"] == "What is robotics?"
    assert page.data[1].id == "message_persist_assistant_001"
    assert page.data[1].content["text"] == "Robotics is the study of robots..."
    assert page.has_more is False

    # Step 5: Update an existing message
    updated_message = ThreadItem.model_validate({
        "id": "message_persist_assistant_001",
        "type": "message",
        "role": "assistant",
        "content": {"text": "Robotics is the interdisciplinary study of robots and automation."},
        "created_at": assistant_message.created_at  # Keep original timestamp
    })

    await store.save_item("thread_test_persist_001", updated_message, mock_context)

    # Step 6: Verify the update
    loaded_message = await store.load_item("thread_test_persist_001", "message_persist_assistant_001", mock_context)
    assert loaded_message.content["text"] == "Robotics is the interdisciplinary study of robots and automation."

    # Step 7: Verify message count unchanged
    page_after_update = await store.load_thread_items("thread_test_persist_001", None, 50, "asc", mock_context)
    assert len(page_after_update.data) == 2  # Still 2 messages, not 3


@pytest.mark.asyncio
async def test_content_size_validation(store, mock_context):
    """
    Test that messages exceeding 32KB are rejected.

    This test will fail until T028 is implemented.
    """
    # Create a thread
    thread = ThreadMetadata(
        id="thread_test_size_001",
        created_at=datetime.now(timezone.utc),
        updated_at=datetime.now(timezone.utc),
        metadata={}
    )

    await store.save_thread(thread, mock_context)

    # Try to add a message with content exceeding 32KB
    large_content = {"text": "x" * 40000}  # Exceeds 32KB limit

    large_message = ThreadItem.model_validate({
        "id": "message_large_001",
        "type": "message",
        "role": "user",
        "content": large_content,
        "created_at": datetime.now(timezone.utc)
    })

    # This should raise a validation error
    with pytest.raises(Exception):  # Could be ValueError or database constraint error
        await store.add_thread_item("thread_test_size_001", large_message, mock_context)
