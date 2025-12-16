# backend/tests/unit/services/test_translation_service.py
"""
Unit tests for translation service.

Tests:
- Content hash computation
- Quota management (check, update, status)
- History operations (save, get)
- Content translation
"""

import pytest
from unittest.mock import MagicMock, patch, AsyncMock
from datetime import date, datetime, timezone, timedelta
from contextlib import contextmanager

from src.services.translation_service import (
    TranslationService,
    compute_content_hash,
    DAILY_QUOTA_LIMIT,
    MIN_CONTENT_LENGTH,
)
from src.models.translation import (
    QuotaStatusResponse,
    TranslationHistoryResponse,
)


class MockCursor:
    """Mock database cursor."""

    def __init__(self):
        self.executed = []
        self.fetchone_result = None
        self.fetchall_result = []

    def execute(self, query, params=None):
        self.executed.append((query, params))

    def fetchone(self):
        return self.fetchone_result

    def fetchall(self):
        return self.fetchall_result


class MockConnection:
    """Mock database connection."""

    def __init__(self):
        self.cursor_instance = MockCursor()
        self.committed = False

    @contextmanager
    def cursor(self):
        yield self.cursor_instance

    def commit(self):
        self.committed = True


class TestComputeContentHash:
    """Tests for compute_content_hash function."""

    def test_computes_hash_for_content(self):
        """Should compute SHA-256 hash for content."""
        content = "# Introduction\n\nThis is some markdown content."

        result = compute_content_hash(content)

        assert result is not None
        assert len(result) == 64  # SHA-256 hex string length
        assert all(c in '0123456789abcdef' for c in result)

    def test_same_content_produces_same_hash(self):
        """Same content should always produce the same hash."""
        content = "Some markdown content"

        hash1 = compute_content_hash(content)
        hash2 = compute_content_hash(content)

        assert hash1 == hash2

    def test_different_content_produces_different_hashes(self):
        """Different content should produce different hashes."""
        content1 = "Content version 1"
        content2 = "Content version 2"

        hash1 = compute_content_hash(content1)
        hash2 = compute_content_hash(content2)

        assert hash1 != hash2

    def test_handles_empty_string(self):
        """Should handle empty string."""
        result = compute_content_hash("")

        assert result is not None
        assert len(result) == 64

    def test_handles_unicode_content(self):
        """Should handle Unicode/Urdu content."""
        content = "# اردو متن\n\nیہ اردو میں ہے۔"

        result = compute_content_hash(content)

        assert result is not None
        assert len(result) == 64


class TestTranslationServiceQuota:
    """Tests for TranslationService quota operations."""

    @pytest.fixture
    def service(self):
        """Create TranslationService instance."""
        return TranslationService()

    @pytest.fixture
    def mock_connection(self):
        """Create mock database connection."""
        return MockConnection()

    @pytest.mark.asyncio
    async def test_get_quota_status_for_authenticated_user(self, service, mock_connection):
        """Should return quota status for authenticated user."""
        mock_connection.cursor_instance.fetchone_result = (3,)  # 3 used

        with patch('src.services.translation_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            result = await service.get_quota_status(user_id='user123')

        assert isinstance(result, QuotaStatusResponse)
        assert result.limit == DAILY_QUOTA_LIMIT
        assert result.used == 3
        assert result.remaining == 2

    @pytest.mark.asyncio
    async def test_get_quota_status_for_anonymous_user(self, service, mock_connection):
        """Should return quota status for anonymous user by IP hash."""
        mock_connection.cursor_instance.fetchone_result = (2,)  # 2 used

        with patch('src.services.translation_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            result = await service.get_quota_status(ip_hash='abc123hash')

        assert result.used == 2
        assert result.remaining == 3

    @pytest.mark.asyncio
    async def test_get_quota_status_with_no_record(self, service, mock_connection):
        """Should return zero usage when no record exists."""
        mock_connection.cursor_instance.fetchone_result = None

        with patch('src.services.translation_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            result = await service.get_quota_status(user_id='user123')

        assert result.used == 0
        assert result.remaining == DAILY_QUOTA_LIMIT

    @pytest.mark.asyncio
    async def test_get_quota_status_resets_at_midnight(self, service, mock_connection):
        """Should return correct reset time (midnight UTC next day)."""
        mock_connection.cursor_instance.fetchone_result = (1,)

        with patch('src.services.translation_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            result = await service.get_quota_status(user_id='user123')

        # Reset time should be midnight UTC tomorrow
        tomorrow = date.today() + timedelta(days=1)
        expected_reset = datetime.combine(tomorrow, datetime.min.time()).replace(tzinfo=timezone.utc)

        assert result.resets_at == expected_reset

    @pytest.mark.asyncio
    async def test_check_and_update_quota_success_authenticated(self, service, mock_connection):
        """Should successfully update quota for authenticated user."""
        mock_connection.cursor_instance.fetchone_result = (2,)  # Now at 2 used after increment

        with patch('src.services.translation_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            success, remaining = await service.check_and_update_quota(user_id='user123')

        assert success is True
        assert remaining == 3  # 5 - 2 = 3

    @pytest.mark.asyncio
    async def test_check_and_update_quota_success_anonymous(self, service, mock_connection):
        """Should successfully update quota for anonymous user by IP hash."""
        mock_connection.cursor_instance.fetchone_result = (1,)  # Now at 1 used after increment

        with patch('src.services.translation_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            success, remaining = await service.check_and_update_quota(ip_hash='abc123hash')

        assert success is True
        assert remaining == 4  # 5 - 1 = 4

    @pytest.mark.asyncio
    async def test_check_and_update_quota_exhausted(self, service, mock_connection):
        """Should fail when quota is exhausted."""
        mock_connection.cursor_instance.fetchone_result = None  # No update happened

        with patch('src.services.translation_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            success, remaining = await service.check_and_update_quota(user_id='user123')

        assert success is False
        assert remaining == 0


class TestTranslationServiceHistory:
    """Tests for TranslationService history operations."""

    @pytest.fixture
    def service(self):
        """Create TranslationService instance."""
        return TranslationService()

    @pytest.fixture
    def mock_connection(self):
        """Create mock database connection."""
        return MockConnection()

    @pytest.mark.asyncio
    async def test_save_translation_history_authenticated(self, service, mock_connection):
        """Should save history for authenticated user."""
        with patch('src.services.translation_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            await service.save_translation_history(
                user_id='user123',
                ip_hash=None,
                page_url='/docs/intro',
                original_content_hash='a' * 64,
            )

        # Verify INSERT/UPDATE was executed
        assert len(mock_connection.cursor_instance.executed) == 1
        query = mock_connection.cursor_instance.executed[0][0]
        assert 'INSERT INTO translation_history' in query
        assert 'user_id' in query

    @pytest.mark.asyncio
    async def test_save_translation_history_anonymous(self, service, mock_connection):
        """Should save history for anonymous user by IP hash."""
        with patch('src.services.translation_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            await service.save_translation_history(
                user_id=None,
                ip_hash='abc123hash',
                page_url='/docs/intro',
                original_content_hash='a' * 64,
            )

        # Verify INSERT/UPDATE was executed
        assert len(mock_connection.cursor_instance.executed) == 1
        query = mock_connection.cursor_instance.executed[0][0]
        assert 'INSERT INTO translation_history' in query
        assert 'ip_hash' in query

    @pytest.mark.asyncio
    async def test_get_translation_history_found(self, service, mock_connection):
        """Should return translation history when found."""
        mock_time = datetime(2025, 1, 1, 12, 0, 0, tzinfo=timezone.utc)
        mock_connection.cursor_instance.fetchone_result = ('a' * 64, mock_time)

        with patch('src.services.translation_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            result = await service.get_translation_history(
                page_url='/docs/intro',
                user_id='user123',
            )

        assert isinstance(result, TranslationHistoryResponse)
        assert result.has_translation is True
        assert result.original_content_hash == 'a' * 64
        assert result.translated_at == mock_time

    @pytest.mark.asyncio
    async def test_get_translation_history_not_found(self, service, mock_connection):
        """Should return not found when no history."""
        mock_connection.cursor_instance.fetchone_result = None

        with patch('src.services.translation_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            result = await service.get_translation_history(
                page_url='/docs/intro',
                user_id='user123',
            )

        assert result.has_translation is False

    @pytest.mark.asyncio
    async def test_get_translation_history_content_changed(self, service, mock_connection):
        """Should detect when content has changed since translation."""
        mock_time = datetime(2025, 1, 1, 12, 0, 0, tzinfo=timezone.utc)
        mock_connection.cursor_instance.fetchone_result = ('old_hash' + 'a' * 56, mock_time)

        with patch('src.services.translation_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            result = await service.get_translation_history(
                page_url='/docs/intro',
                user_id='user123',
                current_content_hash='new_hash' + 'a' * 56,  # Different hash
            )

        assert result.has_translation is True
        assert result.content_changed is True


class TestTranslationServiceTranslate:
    """Tests for TranslationService translate_content operation."""

    @pytest.fixture
    def service(self):
        """Create TranslationService instance."""
        return TranslationService()

    @pytest.fixture
    def mock_connection(self):
        """Create mock database connection."""
        return MockConnection()

    @pytest.mark.asyncio
    async def test_translate_content_success(self, service, mock_connection):
        """Should successfully translate content."""
        content = "a" * 100  # Min 50 chars required

        with patch('src.services.translation_service.get_db_connection') as mock_db, \
             patch('src.services.translation_service.translate_to_urdu', new_callable=AsyncMock) as mock_translate:

            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)
            mock_translate.return_value = ("Translated content here", 1500)

            translated, time_ms, content_hash = await service.translate_content(
                content=content,
                page_url='/docs/intro',
                user_id='user123',
            )

        assert translated == "Translated content here"
        assert time_ms == 1500
        assert len(content_hash) == 64

    @pytest.mark.asyncio
    async def test_translate_content_rejects_short_content(self, service):
        """Should reject content shorter than minimum length."""
        content = "Too short"  # Less than 50 chars

        with pytest.raises(ValueError) as exc_info:
            await service.translate_content(
                content=content,
                page_url='/docs/intro',
                user_id='user123',
            )

        assert "too short" in str(exc_info.value).lower()

    @pytest.mark.asyncio
    async def test_translate_content_with_anonymous_user(self, service, mock_connection):
        """Should translate content for anonymous user by IP hash."""
        content = "a" * 100

        with patch('src.services.translation_service.get_db_connection') as mock_db, \
             patch('src.services.translation_service.translate_to_urdu', new_callable=AsyncMock) as mock_translate:

            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)
            mock_translate.return_value = ("Translated content", 2000)

            translated, time_ms, content_hash = await service.translate_content(
                content=content,
                page_url='/docs/intro',
                ip_hash='abc123hash',
            )

        assert translated == "Translated content"
        assert time_ms == 2000


class TestConstants:
    """Tests for module constants."""

    def test_daily_quota_limit_is_five(self):
        """Daily quota limit should be 5."""
        assert DAILY_QUOTA_LIMIT == 5

    def test_min_content_length_is_50(self):
        """Minimum content length should be 50 characters."""
        assert MIN_CONTENT_LENGTH == 50
