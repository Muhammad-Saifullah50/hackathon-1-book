"""
Unit tests for NeonChatKitStore implementation.

Following TDD Red-Green-Refactor cycle per constitution mandate.
"""

import pytest
import pytest_asyncio
from unittest.mock import Mock, AsyncMock, patch
from datetime import datetime, timezone
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))

from chatkit.types import ThreadMetadata, UserMessageItem
from src.services.chat_store import NeonChatKitStore


class TestNeonChatKitStore:
    """Test suite for NeonChatKitStore class."""

    @pytest_asyncio.fixture
    async def store(self):
        """Create store instance for testing with mocked pool."""
        # Use a mock connection string for unit tests
        store_instance = NeonChatKitStore("postgresql://test:test@localhost/test")

        # Mock the connection pool to avoid real database connections
        mock_pool = AsyncMock()
        mock_conn = AsyncMock()
        mock_pool.acquire.return_value.__aenter__.return_value = mock_conn
        mock_pool.acquire.return_value.__aexit__.return_value = None

        store_instance._pool = mock_pool

        yield store_instance

        # Cleanup
        await store_instance.close()

    @pytest.fixture
    def mock_context(self):
        """Create mock context for tests."""
        return {"user_id": "user_test123"}

    # T007: Test generate_thread_id
    def test_generate_thread_id_format(self, store, mock_context):
        """Test that generated thread IDs follow the expected format."""
        thread_id = store.generate_thread_id(mock_context)

        assert thread_id.startswith("thread_")
        assert len(thread_id) == len("thread_") + 16  # thread_ + 16 hex chars
        assert all(c in "0123456789abcdef" for c in thread_id.replace("thread_", ""))

    def test_generate_thread_id_unique(self, store, mock_context):
        """Test that generated thread IDs are unique."""
        id1 = store.generate_thread_id(mock_context)
        id2 = store.generate_thread_id(mock_context)

        assert id1 != id2

    # T008: Test generate_item_id
    def test_generate_item_id_format_message(self, store, mock_context):
        """Test that generated item IDs follow the expected format for messages."""
        mock_thread = Mock(spec=ThreadMetadata)
        mock_thread.id = "thread_abc123"

        item_id = store.generate_item_id("message", mock_thread, mock_context)

        assert item_id.startswith("message_")
        assert len(item_id) == len("message_") + 12  # message_ + 12 hex chars

    def test_generate_item_id_format_tool_call(self, store, mock_context):
        """Test that generated item IDs respect item type."""
        mock_thread = Mock(spec=ThreadMetadata)
        mock_thread.id = "thread_abc123"

        item_id = store.generate_item_id("tool_call", mock_thread, mock_context)

        assert item_id.startswith("tool_call_")

    def test_generate_item_id_unique(self, store, mock_context):
        """Test that generated item IDs are unique."""
        mock_thread = Mock(spec=ThreadMetadata)
        mock_thread.id = "thread_abc123"

        id1 = store.generate_item_id("message", mock_thread, mock_context)
        id2 = store.generate_item_id("message", mock_thread, mock_context)

        assert id1 != id2

    # T011: Test load_thread
    @pytest.mark.asyncio
    async def test_load_thread_existing(self, store, mock_context):
        """Test loading an existing thread from database."""
        # This test will fail until T014 is implemented
        thread = await store.load_thread("thread_test123", mock_context)

        assert thread.id == "thread_test123"
        assert isinstance(thread, ThreadMetadata)

    @pytest.mark.asyncio
    async def test_load_thread_not_found(self, store, mock_context):
        """Test loading a non-existent thread raises NotFoundError."""
        from chatkit.store import NotFoundError

        with pytest.raises(NotFoundError):
            await store.load_thread("thread_nonexistent", mock_context)

    # T012: Test load_thread_items
    @pytest.mark.asyncio
    async def test_load_thread_items_empty(self, store, mock_context):
        """Test loading items from an empty thread."""
        # This test will fail until T015 is implemented
        page = await store.load_thread_items("thread_test123", None, 50, "asc", mock_context)

        assert page.data == []
        assert page.has_more is False
        assert page.after is None

    @pytest.mark.asyncio
    async def test_load_thread_items_pagination(self, store, mock_context):
        """Test pagination of thread items."""
        # This test will fail until T015 is implemented
        page = await store.load_thread_items("thread_test123", None, 10, "desc", mock_context)

        assert isinstance(page.data, list)
        assert isinstance(page.has_more, bool)
        assert page.after is None or isinstance(page.after, str)

    # T020: Test save_thread
    @pytest.mark.asyncio
    async def test_save_thread_new(self, store, mock_context):
        """Test saving a new thread to database."""
        # This test will fail until T024 is implemented
        thread = ThreadMetadata(
            id="thread_test_new_001",
            created_at=datetime.now(timezone.utc),
            updated_at=datetime.now(timezone.utc),
            metadata={"test": "data"}
        )

        # Mock the database connection execute method
        mock_conn = store._pool.acquire.return_value.__aenter__.return_value
        mock_conn.execute = AsyncMock()

        await store.save_thread(thread, mock_context)

        # Verify execute was called with correct SQL
        assert mock_conn.execute.called
        call_args = mock_conn.execute.call_args[0]
        assert "INSERT INTO chat_threads" in call_args[0]
        assert call_args[1] == "thread_test_new_001"  # thread id

    # T021: Test add_thread_item
    @pytest.mark.asyncio
    async def test_add_thread_item(self, store, mock_context):
        """Test adding a new item to a thread."""
        # This test will fail until T025 is implemented
        item = UserMessageItem(
            id="message_test_001",
            role="user",
            content={"text": "Test message"},
            created_at=datetime.now(timezone.utc)
        )

        # Mock the database connection execute method
        mock_conn = store._pool.acquire.return_value.__aenter__.return_value
        mock_conn.execute = AsyncMock()

        await store.add_thread_item("thread_test123", item, mock_context)

        # Verify execute was called with correct SQL
        assert mock_conn.execute.called
        call_args = mock_conn.execute.call_args[0]
        assert "INSERT INTO chat_thread_items" in call_args[0]
        assert call_args[1] == "message_test_001"  # item id
        assert call_args[2] == "thread_test123"  # thread id

    # T022: Test save_item
    @pytest.mark.asyncio
    async def test_save_item_update(self, store, mock_context):
        """Test updating an existing item."""
        from chatkit.types import AssistantMessageItem

        # This test will fail until T026 is implemented
        item = AssistantMessageItem(
            id="message_test_002",
            role="assistant",
            content={"text": "Updated response"},
            created_at=datetime.now(timezone.utc)
        )

        # Mock the database connection execute method
        mock_conn = store._pool.acquire.return_value.__aenter__.return_value
        mock_conn.execute = AsyncMock()

        await store.save_item("thread_test123", item, mock_context)

        # Verify execute was called with correct SQL (upsert)
        assert mock_conn.execute.called
        call_args = mock_conn.execute.call_args[0]
        assert "INSERT INTO chat_thread_items" in call_args[0]
        assert "ON CONFLICT" in call_args[0]
        assert call_args[1] == "message_test_002"  # item id
