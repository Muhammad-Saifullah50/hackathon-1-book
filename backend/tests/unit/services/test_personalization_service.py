# backend/tests/unit/services/test_personalization_service.py
"""
Unit tests for personalization service.

Tests:
- Profile hash computation
- Content hash computation
- Quota management (check, decrement, status)
- History operations (get, upsert, page check)
- Free re-personalization detection
"""

import pytest
from unittest.mock import MagicMock, patch, AsyncMock
from datetime import date, datetime, timezone, timedelta
from contextlib import contextmanager

from src.services.personalization_service import (
    PersonalizationService,
    compute_profile_hash,
    compute_content_hash,
    DAILY_QUOTA_LIMIT,
    MIN_CONTENT_LENGTH,
)
from src.models.personalization import (
    QuotaStatus,
    PersonalizationHistoryItem,
    PersonalizationHistoryResponse,
    PageHistoryResponse,
)


class MockCursor:
    """Mock database cursor."""

    def __init__(self):
        self.executed = []
        self.results = []
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
        self.rolledback = False

    @contextmanager
    def cursor(self):
        yield self.cursor_instance

    def commit(self):
        self.committed = True

    def rollback(self):
        self.rolledback = True


class TestComputeProfileHash:
    """Tests for compute_profile_hash function."""

    def test_computes_hash_for_complete_profile(self):
        """Should compute SHA-256 hash for a complete profile."""
        profile = {
            'tech_background': 'intermediate',
            'learning_mode': 'visual',
            'learning_speed': 'balanced',
            'preferred_language': 'en',
            'education_level': 'bachelors',
            'primary_goal': 'career',
            'focus_area': 'software',
        }

        result = compute_profile_hash(profile)

        assert result is not None
        assert len(result) == 64  # SHA-256 hex string length
        assert all(c in '0123456789abcdef' for c in result)

    def test_computes_hash_for_minimal_profile(self):
        """Should handle profiles with missing optional fields."""
        profile = {
            'tech_background': 'beginner',
        }

        result = compute_profile_hash(profile)

        assert result is not None
        assert len(result) == 64

    def test_same_profile_produces_same_hash(self):
        """Same profile should always produce the same hash."""
        profile = {
            'tech_background': 'advanced',
            'learning_mode': 'hands_on',
            'learning_speed': 'accelerated',
        }

        hash1 = compute_profile_hash(profile)
        hash2 = compute_profile_hash(profile)

        assert hash1 == hash2

    def test_different_profiles_produce_different_hashes(self):
        """Different profiles should produce different hashes."""
        profile1 = {'tech_background': 'beginner'}
        profile2 = {'tech_background': 'advanced'}

        hash1 = compute_profile_hash(profile1)
        hash2 = compute_profile_hash(profile2)

        assert hash1 != hash2

    def test_uses_default_language_when_not_provided(self):
        """Should use 'en' as default preferred_language."""
        profile_with_lang = {'tech_background': 'beginner', 'preferred_language': 'en'}
        profile_without_lang = {'tech_background': 'beginner'}

        hash1 = compute_profile_hash(profile_with_lang)
        hash2 = compute_profile_hash(profile_without_lang)

        assert hash1 == hash2

    def test_ignores_non_personalization_fields(self):
        """Should ignore fields not relevant to personalization."""
        profile1 = {'tech_background': 'beginner', 'user_id': '123', 'email': 'test@test.com'}
        profile2 = {'tech_background': 'beginner', 'user_id': '456', 'email': 'other@test.com'}

        hash1 = compute_profile_hash(profile1)
        hash2 = compute_profile_hash(profile2)

        assert hash1 == hash2


class TestComputeContentHash:
    """Tests for compute_content_hash function."""

    def test_computes_hash_for_content(self):
        """Should compute SHA-256 hash for content."""
        content = "# Introduction\n\nThis is some markdown content."

        result = compute_content_hash(content)

        assert result is not None
        assert len(result) == 64

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


class TestPersonalizationServiceQuota:
    """Tests for PersonalizationService quota operations."""

    @pytest.fixture
    def service(self):
        """Create PersonalizationService instance."""
        return PersonalizationService()

    @pytest.fixture
    def mock_connection(self):
        """Create mock database connection."""
        return MockConnection()

    @pytest.mark.asyncio
    async def test_get_quota_status_with_existing_record(self, service, mock_connection):
        """Should return quota status for existing user."""
        mock_connection.cursor_instance.fetchone_result = (3,)  # 3 used

        with patch('src.services.personalization_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            result = await service.get_quota_status('user123')

        assert isinstance(result, QuotaStatus)
        assert result.limit == DAILY_QUOTA_LIMIT
        assert result.used == 3
        assert result.remaining == 2

    @pytest.mark.asyncio
    async def test_get_quota_status_with_no_record(self, service, mock_connection):
        """Should return zero usage when no record exists."""
        mock_connection.cursor_instance.fetchone_result = None

        with patch('src.services.personalization_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            result = await service.get_quota_status('user123')

        assert result.used == 0
        assert result.remaining == DAILY_QUOTA_LIMIT

    @pytest.mark.asyncio
    async def test_get_quota_status_resets_at_midnight(self, service, mock_connection):
        """Should return correct reset time (midnight UTC next day)."""
        mock_connection.cursor_instance.fetchone_result = (1,)

        with patch('src.services.personalization_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            result = await service.get_quota_status('user123')

        # Reset time should be midnight UTC tomorrow
        tomorrow = date.today() + timedelta(days=1)
        expected_reset = datetime.combine(tomorrow, datetime.min.time()).replace(tzinfo=timezone.utc)

        assert result.resets_at == expected_reset

    @pytest.mark.asyncio
    async def test_check_and_decrement_quota_success(self, service, mock_connection):
        """Should successfully decrement quota when under limit."""
        mock_connection.cursor_instance.fetchone_result = (2,)  # Now at 2 used after increment

        with patch('src.services.personalization_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            success, remaining = await service.check_and_decrement_quota('user123')

        assert success is True
        assert remaining == 3  # 5 - 2 = 3

    @pytest.mark.asyncio
    async def test_check_and_decrement_quota_exhausted(self, service, mock_connection):
        """Should fail when quota is exhausted."""
        # First query returns None (no update happened), second returns current count
        mock_connection.cursor_instance.fetchone_result = None

        def side_effect(query, params=None):
            if 'UPDATE' in query or 'INSERT' in query:
                mock_connection.cursor_instance.fetchone_result = None
            elif 'SELECT' in query:
                mock_connection.cursor_instance.fetchone_result = (5,)  # At limit

        mock_connection.cursor_instance.execute = side_effect

        with patch('src.services.personalization_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            success, remaining = await service.check_and_decrement_quota('user123')

        assert success is False
        assert remaining == 0


class TestPersonalizationServiceHistory:
    """Tests for PersonalizationService history operations."""

    @pytest.fixture
    def service(self):
        """Create PersonalizationService instance."""
        return PersonalizationService()

    @pytest.fixture
    def mock_connection(self):
        """Create mock database connection."""
        return MockConnection()

    @pytest.mark.asyncio
    async def test_upsert_history_creates_record(self, service, mock_connection):
        """Should create new history record."""
        with patch('src.services.personalization_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            await service.upsert_history(
                user_id='user123',
                page_url='/docs/intro',
                profile_hash='a' * 64,
                original_content_hash='b' * 64
            )

        # Verify INSERT/UPDATE was executed
        assert len(mock_connection.cursor_instance.executed) == 1
        assert 'INSERT INTO personalization_history' in mock_connection.cursor_instance.executed[0][0]

    @pytest.mark.asyncio
    async def test_get_history_returns_pages(self, service, mock_connection):
        """Should return list of personalized pages."""
        mock_time = datetime(2025, 1, 1, 12, 0, 0, tzinfo=timezone.utc)
        mock_connection.cursor_instance.fetchall_result = [
            ('/docs/intro', 'a' * 64, mock_time),
            ('/docs/chapter1', 'b' * 64, mock_time),
        ]

        with patch('src.services.personalization_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            result = await service.get_history('user123', 'current_hash')

        assert isinstance(result, PersonalizationHistoryResponse)
        assert len(result.pages) == 2
        assert result.pages[0].url == '/docs/intro'
        assert result.current_profile_hash == 'current_hash'

    @pytest.mark.asyncio
    async def test_get_history_empty_when_no_records(self, service, mock_connection):
        """Should return empty list when no history."""
        mock_connection.cursor_instance.fetchall_result = []

        with patch('src.services.personalization_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            result = await service.get_history('user123', 'current_hash')

        assert len(result.pages) == 0

    @pytest.mark.asyncio
    async def test_get_page_history_found(self, service, mock_connection):
        """Should return page history when found."""
        mock_time = datetime(2025, 1, 1, 12, 0, 0, tzinfo=timezone.utc)
        mock_connection.cursor_instance.fetchone_result = ('a' * 64, 'b' * 64, mock_time)

        with patch('src.services.personalization_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            result = await service.get_page_history('user123', '/docs/intro', 'a' * 64)

        assert isinstance(result, PageHistoryResponse)
        assert result.found is True
        assert result.profile_hash == 'a' * 64
        assert result.is_stale is False  # Same hash

    @pytest.mark.asyncio
    async def test_get_page_history_stale_when_profile_changed(self, service, mock_connection):
        """Should mark as stale when profile hash differs."""
        mock_time = datetime(2025, 1, 1, 12, 0, 0, tzinfo=timezone.utc)
        mock_connection.cursor_instance.fetchone_result = ('old_hash' + 'a' * 56, 'b' * 64, mock_time)

        with patch('src.services.personalization_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            result = await service.get_page_history('user123', '/docs/intro', 'new_hash' + 'a' * 56)

        assert result.found is True
        assert result.is_stale is True  # Different hash

    @pytest.mark.asyncio
    async def test_get_page_history_not_found(self, service, mock_connection):
        """Should return not found when no history."""
        mock_connection.cursor_instance.fetchone_result = None

        with patch('src.services.personalization_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            result = await service.get_page_history('user123', '/docs/intro', 'current_hash')

        assert result.found is False


class TestPersonalizationServiceFreeRepersonalization:
    """Tests for free re-personalization detection."""

    @pytest.fixture
    def service(self):
        """Create PersonalizationService instance."""
        return PersonalizationService()

    @pytest.fixture
    def mock_connection(self):
        """Create mock database connection."""
        return MockConnection()

    @pytest.mark.asyncio
    async def test_is_free_repersonalization_true_when_exists(self, service, mock_connection):
        """Should return True when page was previously personalized."""
        mock_connection.cursor_instance.fetchone_result = (1,)  # Record exists

        with patch('src.services.personalization_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            result = await service.is_free_repersonalization('user123', '/docs/intro')

        assert result is True

    @pytest.mark.asyncio
    async def test_is_free_repersonalization_false_when_not_exists(self, service, mock_connection):
        """Should return False when page was never personalized."""
        mock_connection.cursor_instance.fetchone_result = None

        with patch('src.services.personalization_service.get_db_connection') as mock_db:
            mock_db.return_value.__enter__ = MagicMock(return_value=mock_connection)
            mock_db.return_value.__exit__ = MagicMock(return_value=None)

            result = await service.is_free_repersonalization('user123', '/docs/intro')

        assert result is False

    @pytest.mark.asyncio
    async def test_is_free_repersonalization_handles_errors(self, service, mock_connection):
        """Should return False on database errors."""
        with patch('src.services.personalization_service.get_db_connection') as mock_db:
            mock_db.side_effect = Exception("Database error")

            result = await service.is_free_repersonalization('user123', '/docs/intro')

        assert result is False


class TestConstants:
    """Tests for module constants."""

    def test_daily_quota_limit_is_five(self):
        """Daily quota limit should be 5."""
        assert DAILY_QUOTA_LIMIT == 5

    def test_min_content_length_is_100(self):
        """Minimum content length should be 100 characters."""
        assert MIN_CONTENT_LENGTH == 100
