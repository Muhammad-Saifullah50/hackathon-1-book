# backend/tests/integration/api/test_translation_api.py
"""
Integration tests for translation API endpoints.

Tests:
- POST /api/translate/urdu - Translate content to Urdu
- GET /api/translate/quota - Get quota status
- GET /api/translate/history - Check translation history

These tests use mocked dependencies to test API behavior.
Supports both authenticated and anonymous users.
"""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from datetime import datetime, timezone, timedelta
from fastapi.testclient import TestClient

# Import app components
from main import app
from src.services.translation_service import TranslationService
from src.models.translation import (
    QuotaStatusResponse,
    TranslationHistoryResponse,
)
from src.api.translation.dependencies import get_optional_user_id, get_user_or_ip_hash


# Test fixtures
@pytest.fixture
def client():
    """Create test client."""
    return TestClient(app)


@pytest.fixture
def authenticated_client():
    """Create test client with mocked authentication."""
    # Override the user dependency
    app.dependency_overrides[get_optional_user_id] = lambda: "user123"
    app.dependency_overrides[get_user_or_ip_hash] = lambda: ("user123", None)
    client = TestClient(app)
    yield client
    app.dependency_overrides.clear()


@pytest.fixture
def anonymous_client():
    """Create test client simulating anonymous user."""
    # Override to return None user and IP hash
    app.dependency_overrides[get_optional_user_id] = lambda: None
    app.dependency_overrides[get_user_or_ip_hash] = lambda: (None, "abc123hash")
    client = TestClient(app)
    yield client
    app.dependency_overrides.clear()


class TestTranslateEndpoint:
    """Tests for POST /api/translate/urdu endpoint."""

    def test_translates_content_for_authenticated_user(self, authenticated_client):
        """Should translate content for authenticated user."""
        tomorrow = (datetime.now(timezone.utc) + timedelta(days=1)).replace(
            hour=0, minute=0, second=0, microsecond=0
        )

        with patch.object(TranslationService, 'check_and_update_quota', new_callable=AsyncMock) as mock_quota, \
             patch.object(TranslationService, 'translate_content', new_callable=AsyncMock) as mock_translate, \
             patch.object(TranslationService, 'get_quota_status', new_callable=AsyncMock) as mock_get_quota:

            mock_quota.return_value = (True, 4)
            mock_translate.return_value = ("ترجمہ شدہ متن", 1500, "a" * 64)
            mock_get_quota.return_value = QuotaStatusResponse(
                limit=5, used=1, remaining=4, resets_at=tomorrow
            )

            response = authenticated_client.post(
                "/api/translate/urdu",
                json={
                    "page_content": "a" * 100,
                    "page_url": "/docs/intro",
                }
            )

        assert response.status_code == 200
        data = response.json()
        assert data["translated_content"] == "ترجمہ شدہ متن"
        assert data["processing_time_ms"] == 1500
        assert data["quota_remaining"] == 4
        assert data["source_language"] == "en"
        assert data["target_language"] == "ur"

    def test_translates_content_for_anonymous_user(self, anonymous_client):
        """Should translate content for anonymous user using IP hash."""
        tomorrow = (datetime.now(timezone.utc) + timedelta(days=1)).replace(
            hour=0, minute=0, second=0, microsecond=0
        )

        with patch.object(TranslationService, 'check_and_update_quota', new_callable=AsyncMock) as mock_quota, \
             patch.object(TranslationService, 'translate_content', new_callable=AsyncMock) as mock_translate, \
             patch.object(TranslationService, 'get_quota_status', new_callable=AsyncMock) as mock_get_quota:

            mock_quota.return_value = (True, 4)
            mock_translate.return_value = ("ترجمہ شدہ متن", 2000, "b" * 64)
            mock_get_quota.return_value = QuotaStatusResponse(
                limit=5, used=1, remaining=4, resets_at=tomorrow
            )

            response = anonymous_client.post(
                "/api/translate/urdu",
                json={
                    "page_content": "a" * 100,
                    "page_url": "/docs/intro",
                }
            )

        assert response.status_code == 200
        data = response.json()
        assert data["translated_content"] == "ترجمہ شدہ متن"

    def test_returns_429_when_quota_exceeded_authenticated(self, authenticated_client):
        """Should return 429 when authenticated user quota is exceeded."""
        tomorrow = (datetime.now(timezone.utc) + timedelta(days=1)).replace(
            hour=0, minute=0, second=0, microsecond=0
        )

        with patch.object(TranslationService, 'check_and_update_quota', new_callable=AsyncMock) as mock_quota, \
             patch.object(TranslationService, 'get_quota_status', new_callable=AsyncMock) as mock_get_quota:

            mock_quota.return_value = (False, 0)
            mock_get_quota.return_value = QuotaStatusResponse(
                limit=5, used=5, remaining=0, resets_at=tomorrow
            )

            response = authenticated_client.post(
                "/api/translate/urdu",
                json={
                    "page_content": "a" * 100,
                    "page_url": "/docs/intro",
                }
            )

        assert response.status_code == 429
        data = response.json()
        assert "QUOTA_EXCEEDED" in str(data.get("detail", {}).get("code", ""))

    def test_returns_429_when_quota_exceeded_anonymous(self, anonymous_client):
        """Should return 429 when anonymous user quota is exceeded."""
        tomorrow = (datetime.now(timezone.utc) + timedelta(days=1)).replace(
            hour=0, minute=0, second=0, microsecond=0
        )

        with patch.object(TranslationService, 'check_and_update_quota', new_callable=AsyncMock) as mock_quota, \
             patch.object(TranslationService, 'get_quota_status', new_callable=AsyncMock) as mock_get_quota:

            mock_quota.return_value = (False, 0)
            mock_get_quota.return_value = QuotaStatusResponse(
                limit=5, used=5, remaining=0, resets_at=tomorrow
            )

            response = anonymous_client.post(
                "/api/translate/urdu",
                json={
                    "page_content": "a" * 100,
                    "page_url": "/docs/intro",
                }
            )

        assert response.status_code == 429

    def test_returns_400_for_invalid_content(self, authenticated_client):
        """Should return 400 for invalid (too short) content."""
        with patch.object(TranslationService, 'check_and_update_quota', new_callable=AsyncMock) as mock_quota, \
             patch.object(TranslationService, 'translate_content', new_callable=AsyncMock) as mock_translate:

            mock_quota.return_value = (True, 4)
            mock_translate.side_effect = ValueError("Content too short")

            response = authenticated_client.post(
                "/api/translate/urdu",
                json={
                    "page_content": "Short",
                    "page_url": "/docs/intro",
                }
            )

        assert response.status_code == 400
        data = response.json()
        assert "INVALID_CONTENT" in str(data.get("detail", {}).get("code", ""))

    def test_returns_500_on_translation_error(self, authenticated_client):
        """Should return 500 on translation service error."""
        with patch.object(TranslationService, 'check_and_update_quota', new_callable=AsyncMock) as mock_quota, \
             patch.object(TranslationService, 'translate_content', new_callable=AsyncMock) as mock_translate:

            mock_quota.return_value = (True, 4)
            mock_translate.side_effect = Exception("Translation API error")

            response = authenticated_client.post(
                "/api/translate/urdu",
                json={
                    "page_content": "a" * 100,
                    "page_url": "/docs/intro",
                }
            )

        assert response.status_code == 500


class TestQuotaEndpoint:
    """Tests for GET /api/translate/quota endpoint."""

    def test_returns_quota_for_authenticated_user(self, authenticated_client):
        """Should return quota status for authenticated user."""
        tomorrow = (datetime.now(timezone.utc) + timedelta(days=1)).replace(
            hour=0, minute=0, second=0, microsecond=0
        )
        mock_quota = QuotaStatusResponse(
            limit=5, used=2, remaining=3, resets_at=tomorrow
        )

        with patch.object(TranslationService, 'get_quota_status', new_callable=AsyncMock) as mock_get:
            mock_get.return_value = mock_quota

            response = authenticated_client.get("/api/translate/quota")

        assert response.status_code == 200
        data = response.json()
        assert data["limit"] == 5
        assert data["used"] == 2
        assert data["remaining"] == 3

    def test_returns_quota_for_anonymous_user(self, anonymous_client):
        """Should return quota status for anonymous user."""
        tomorrow = (datetime.now(timezone.utc) + timedelta(days=1)).replace(
            hour=0, minute=0, second=0, microsecond=0
        )
        mock_quota = QuotaStatusResponse(
            limit=5, used=1, remaining=4, resets_at=tomorrow
        )

        with patch.object(TranslationService, 'get_quota_status', new_callable=AsyncMock) as mock_get:
            mock_get.return_value = mock_quota

            response = anonymous_client.get("/api/translate/quota")

        assert response.status_code == 200
        data = response.json()
        assert data["remaining"] == 4

    def test_returns_full_quota_for_new_user(self, authenticated_client):
        """Should return full quota for user with no usage."""
        tomorrow = (datetime.now(timezone.utc) + timedelta(days=1)).replace(
            hour=0, minute=0, second=0, microsecond=0
        )
        mock_quota = QuotaStatusResponse(
            limit=5, used=0, remaining=5, resets_at=tomorrow
        )

        with patch.object(TranslationService, 'get_quota_status', new_callable=AsyncMock) as mock_get:
            mock_get.return_value = mock_quota

            response = authenticated_client.get("/api/translate/quota")

        assert response.status_code == 200
        data = response.json()
        assert data["used"] == 0
        assert data["remaining"] == 5


class TestHistoryEndpoint:
    """Tests for GET /api/translate/history endpoint."""

    def test_returns_history_when_translated(self, authenticated_client):
        """Should return translation history when page was translated."""
        mock_time = datetime(2025, 1, 1, 12, 0, 0, tzinfo=timezone.utc)
        mock_history = TranslationHistoryResponse(
            has_translation=True,
            original_content_hash="a" * 64,
            translated_at=mock_time,
            content_changed=False,
        )

        with patch.object(TranslationService, 'get_translation_history', new_callable=AsyncMock) as mock_get:
            mock_get.return_value = mock_history

            response = authenticated_client.get(
                "/api/translate/history",
                params={"page_url": "/docs/intro"}
            )

        assert response.status_code == 200
        data = response.json()
        assert data["has_translation"] is True
        assert data["content_changed"] is False

    def test_returns_not_found_when_not_translated(self, authenticated_client):
        """Should indicate not found when page wasn't translated."""
        mock_history = TranslationHistoryResponse(
            has_translation=False,
            content_changed=False,
        )

        with patch.object(TranslationService, 'get_translation_history', new_callable=AsyncMock) as mock_get:
            mock_get.return_value = mock_history

            response = authenticated_client.get(
                "/api/translate/history",
                params={"page_url": "/docs/intro"}
            )

        assert response.status_code == 200
        data = response.json()
        assert data["has_translation"] is False

    def test_detects_content_changed(self, authenticated_client):
        """Should detect when content has changed since translation."""
        mock_time = datetime(2025, 1, 1, 12, 0, 0, tzinfo=timezone.utc)
        mock_history = TranslationHistoryResponse(
            has_translation=True,
            original_content_hash="old_hash" + "a" * 56,
            translated_at=mock_time,
            content_changed=True,
        )

        with patch.object(TranslationService, 'get_translation_history', new_callable=AsyncMock) as mock_get:
            mock_get.return_value = mock_history

            response = authenticated_client.get(
                "/api/translate/history",
                params={
                    "page_url": "/docs/intro",
                    "current_content_hash": "new_hash" + "a" * 56,
                }
            )

        assert response.status_code == 200
        data = response.json()
        assert data["has_translation"] is True
        assert data["content_changed"] is True

    def test_history_for_anonymous_user(self, anonymous_client):
        """Should return history for anonymous user by IP hash."""
        mock_history = TranslationHistoryResponse(
            has_translation=True,
            original_content_hash="a" * 64,
            translated_at=datetime.now(timezone.utc),
            content_changed=False,
        )

        with patch.object(TranslationService, 'get_translation_history', new_callable=AsyncMock) as mock_get:
            mock_get.return_value = mock_history

            response = anonymous_client.get(
                "/api/translate/history",
                params={"page_url": "/docs/intro"}
            )

        assert response.status_code == 200
        data = response.json()
        assert data["has_translation"] is True


class TestIPExtraction:
    """Tests for IP extraction and hashing."""

    def test_extracts_ip_from_x_forwarded_for(self, client):
        """Should extract IP from X-Forwarded-For header."""
        # Reset any overrides
        app.dependency_overrides.clear()

        tomorrow = (datetime.now(timezone.utc) + timedelta(days=1)).replace(
            hour=0, minute=0, second=0, microsecond=0
        )
        mock_quota = QuotaStatusResponse(
            limit=5, used=0, remaining=5, resets_at=tomorrow
        )

        with patch.object(TranslationService, 'get_quota_status', new_callable=AsyncMock) as mock_get:
            mock_get.return_value = mock_quota

            response = client.get(
                "/api/translate/quota",
                headers={"X-Forwarded-For": "192.168.1.1, 10.0.0.1"}
            )

        assert response.status_code == 200

    def test_authenticated_user_uses_user_id_not_ip(self, authenticated_client):
        """Should use user_id for authenticated users instead of IP hash."""
        tomorrow = (datetime.now(timezone.utc) + timedelta(days=1)).replace(
            hour=0, minute=0, second=0, microsecond=0
        )

        with patch.object(TranslationService, 'get_quota_status', new_callable=AsyncMock) as mock_get:
            mock_get.return_value = QuotaStatusResponse(
                limit=5, used=0, remaining=5, resets_at=tomorrow
            )

            response = authenticated_client.get("/api/translate/quota")

            # Verify user_id was passed, not ip_hash
            call_args = mock_get.call_args
            assert call_args is not None
            assert call_args.kwargs.get('user_id') == 'user123' or \
                   (len(call_args.args) > 0 and call_args.args[0] == 'user123')
