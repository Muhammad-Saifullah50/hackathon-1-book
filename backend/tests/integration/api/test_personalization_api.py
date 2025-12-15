# backend/tests/integration/api/test_personalization_api.py
"""
Integration tests for personalization API endpoints.

Tests:
- POST /personalization/personalize - Personalize page content
- GET /personalization/quota - Get quota status
- GET /personalization/history - Get personalization history
- GET /personalization/history/{pageUrl} - Check specific page history

These tests use mocked dependencies to test API behavior.
"""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from datetime import datetime, timezone, timedelta
from fastapi.testclient import TestClient

# Import app components
from main import app
from src.api.auth.dependencies import get_current_user, CurrentUser
from src.services.personalization_service import PersonalizationService
from src.services.profile_service import ProfileService
from src.models.personalization import (
    QuotaStatus,
    PersonalizationHistoryItem,
    PersonalizationHistoryResponse,
    PageHistoryResponse,
)


# Test fixtures
@pytest.fixture
def mock_user():
    """Create a mock authenticated user."""
    return CurrentUser(
        id="user123",
        email="test@example.com",
        name="Test User"
    )


@pytest.fixture
def mock_profile():
    """Create a mock user profile."""
    return MagicMock(
        user_id="user123",
        tech_background="intermediate",
        learning_mode="visual",
        learning_speed="balanced",
        preferred_language="en",
        education_level="bachelors",
        primary_goal="career",
        focus_area="software",
        model_dump=lambda: {
            'tech_background': 'intermediate',
            'learning_mode': 'visual',
            'learning_speed': 'balanced',
            'preferred_language': 'en',
            'education_level': 'bachelors',
            'primary_goal': 'career',
            'focus_area': 'software',
        }
    )


@pytest.fixture
def client(mock_user):
    """Create test client with mocked authentication."""
    app.dependency_overrides[get_current_user] = lambda: mock_user
    client = TestClient(app)
    yield client
    app.dependency_overrides.clear()


class TestGetQuotaEndpoint:
    """Tests for GET /personalization/quota endpoint."""

    def test_returns_quota_status(self, client, mock_user):
        """Should return quota status for authenticated user."""
        tomorrow = (datetime.now(timezone.utc) + timedelta(days=1)).replace(
            hour=0, minute=0, second=0, microsecond=0
        )
        mock_quota = QuotaStatus(
            limit=5,
            used=2,
            remaining=3,
            resets_at=tomorrow
        )

        with patch.object(PersonalizationService, 'get_quota_status', new_callable=AsyncMock) as mock_get:
            mock_get.return_value = mock_quota

            response = client.get("/personalization/quota")

        assert response.status_code == 200
        data = response.json()
        assert data["limit"] == 5
        assert data["used"] == 2
        assert data["remaining"] == 3

    def test_requires_authentication(self):
        """Should return 401 without authentication."""
        app.dependency_overrides.clear()
        client = TestClient(app)

        # Override to raise an exception
        def raise_401():
            from fastapi import HTTPException
            raise HTTPException(status_code=401, detail="Not authenticated")

        app.dependency_overrides[get_current_user] = raise_401

        response = client.get("/personalization/quota")

        assert response.status_code == 401
        app.dependency_overrides.clear()


class TestPersonalizeEndpoint:
    """Tests for POST /personalization/personalize endpoint."""

    def test_personalizes_content_successfully(self, client, mock_user, mock_profile):
        """Should return personalized content."""
        with patch.object(ProfileService, 'get_profile', new_callable=AsyncMock) as mock_get_profile, \
             patch.object(PersonalizationService, 'is_free_repersonalization', new_callable=AsyncMock) as mock_is_free, \
             patch.object(PersonalizationService, 'check_and_decrement_quota', new_callable=AsyncMock) as mock_quota, \
             patch.object(PersonalizationService, 'upsert_history', new_callable=AsyncMock) as mock_upsert, \
             patch('src.api.personalization.routes.personalize_content', new_callable=AsyncMock) as mock_personalize:

            mock_get_profile.return_value = mock_profile
            mock_is_free.return_value = False
            mock_quota.return_value = (True, 4)
            mock_personalize.return_value = ("Personalized content here", 1500)

            response = client.post(
                "/personalization/personalize",
                json={
                    "page_url": "/docs/intro",
                    "page_content": "a" * 150,  # Min 100 chars required
                    "is_free_repersonalization": False
                }
            )

        assert response.status_code == 200
        data = response.json()
        assert data["personalized_content"] == "Personalized content here"
        assert data["processing_time_ms"] == 1500
        assert data["quota_remaining"] == 4

    def test_rejects_short_content(self, client, mock_user, mock_profile):
        """Should reject content shorter than minimum length."""
        with patch.object(ProfileService, 'get_profile', new_callable=AsyncMock) as mock_get_profile:
            mock_get_profile.return_value = mock_profile

            response = client.post(
                "/personalization/personalize",
                json={
                    "page_url": "/docs/intro",
                    "page_content": "Short",  # Less than 100 chars
                    "is_free_repersonalization": False
                }
            )

        assert response.status_code == 400
        data = response.json()
        assert "content_too_short" in str(data.get("detail", {}).get("error", ""))

    def test_rejects_without_profile(self, client, mock_user):
        """Should reject request when user has no profile."""
        with patch.object(ProfileService, 'get_profile', new_callable=AsyncMock) as mock_get_profile:
            mock_get_profile.return_value = None

            response = client.post(
                "/personalization/personalize",
                json={
                    "page_url": "/docs/intro",
                    "page_content": "a" * 150,
                    "is_free_repersonalization": False
                }
            )

        assert response.status_code == 403
        data = response.json()
        assert "profile_incomplete" in str(data.get("detail", {}).get("error", ""))

    def test_returns_429_when_quota_exceeded(self, client, mock_user, mock_profile):
        """Should return 429 when daily quota is exceeded."""
        tomorrow = (datetime.now(timezone.utc) + timedelta(days=1)).replace(
            hour=0, minute=0, second=0, microsecond=0
        )

        with patch.object(ProfileService, 'get_profile', new_callable=AsyncMock) as mock_get_profile, \
             patch.object(PersonalizationService, 'is_free_repersonalization', new_callable=AsyncMock) as mock_is_free, \
             patch.object(PersonalizationService, 'check_and_decrement_quota', new_callable=AsyncMock) as mock_quota, \
             patch.object(PersonalizationService, 'get_quota_status', new_callable=AsyncMock) as mock_get_quota:

            mock_get_profile.return_value = mock_profile
            mock_is_free.return_value = False
            mock_quota.return_value = (False, 0)
            mock_get_quota.return_value = QuotaStatus(
                limit=5,
                used=5,
                remaining=0,
                resets_at=tomorrow
            )

            response = client.post(
                "/personalization/personalize",
                json={
                    "page_url": "/docs/intro",
                    "page_content": "a" * 150,
                    "is_free_repersonalization": False
                }
            )

        assert response.status_code == 429
        data = response.json()
        assert "quota_exceeded" in str(data.get("detail", {}).get("error", ""))

    def test_free_repersonalization_skips_quota(self, client, mock_user, mock_profile):
        """Should not decrement quota for free re-personalization."""
        tomorrow = (datetime.now(timezone.utc) + timedelta(days=1)).replace(
            hour=0, minute=0, second=0, microsecond=0
        )

        with patch.object(ProfileService, 'get_profile', new_callable=AsyncMock) as mock_get_profile, \
             patch.object(PersonalizationService, 'is_free_repersonalization', new_callable=AsyncMock) as mock_is_free, \
             patch.object(PersonalizationService, 'check_and_decrement_quota', new_callable=AsyncMock) as mock_quota, \
             patch.object(PersonalizationService, 'get_quota_status', new_callable=AsyncMock) as mock_get_quota, \
             patch.object(PersonalizationService, 'upsert_history', new_callable=AsyncMock) as mock_upsert, \
             patch('src.api.personalization.routes.personalize_content', new_callable=AsyncMock) as mock_personalize:

            mock_get_profile.return_value = mock_profile
            mock_is_free.return_value = True  # Page was previously personalized
            mock_get_quota.return_value = QuotaStatus(
                limit=5,
                used=3,
                remaining=2,
                resets_at=tomorrow
            )
            mock_personalize.return_value = ("Personalized content here", 1500)

            response = client.post(
                "/personalization/personalize",
                json={
                    "page_url": "/docs/intro",
                    "page_content": "a" * 150,
                    "is_free_repersonalization": True
                }
            )

        assert response.status_code == 200
        # check_and_decrement_quota should NOT have been called
        mock_quota.assert_not_called()


class TestGetHistoryEndpoint:
    """Tests for GET /personalization/history endpoint."""

    def test_returns_history_list(self, client, mock_user, mock_profile):
        """Should return list of personalized pages."""
        mock_time = datetime(2025, 1, 1, 12, 0, 0, tzinfo=timezone.utc)
        mock_history = PersonalizationHistoryResponse(
            pages=[
                PersonalizationHistoryItem(url="/docs/intro", profile_hash="a" * 64, created_at=mock_time),
                PersonalizationHistoryItem(url="/docs/chapter1", profile_hash="b" * 64, created_at=mock_time),
            ],
            current_profile_hash="c" * 64
        )

        with patch.object(ProfileService, 'get_profile', new_callable=AsyncMock) as mock_get_profile, \
             patch.object(PersonalizationService, 'get_history', new_callable=AsyncMock) as mock_get_history:

            mock_get_profile.return_value = mock_profile
            mock_get_history.return_value = mock_history

            response = client.get("/personalization/history")

        assert response.status_code == 200
        data = response.json()
        assert len(data["pages"]) == 2
        assert data["pages"][0]["url"] == "/docs/intro"

    def test_returns_empty_when_no_history(self, client, mock_user, mock_profile):
        """Should return empty list when no history."""
        mock_history = PersonalizationHistoryResponse(
            pages=[],
            current_profile_hash="a" * 64
        )

        with patch.object(ProfileService, 'get_profile', new_callable=AsyncMock) as mock_get_profile, \
             patch.object(PersonalizationService, 'get_history', new_callable=AsyncMock) as mock_get_history:

            mock_get_profile.return_value = mock_profile
            mock_get_history.return_value = mock_history

            response = client.get("/personalization/history")

        assert response.status_code == 200
        data = response.json()
        assert len(data["pages"]) == 0


class TestGetPageHistoryEndpoint:
    """Tests for GET /personalization/history/{pageUrl} endpoint."""

    def test_returns_page_history_when_found(self, client, mock_user, mock_profile):
        """Should return page history when page was personalized."""
        mock_time = datetime(2025, 1, 1, 12, 0, 0, tzinfo=timezone.utc)
        mock_page_history = PageHistoryResponse(
            found=True,
            profile_hash="a" * 64,
            original_content_hash="b" * 64,
            created_at=mock_time,
            is_stale=False
        )

        with patch.object(ProfileService, 'get_profile', new_callable=AsyncMock) as mock_get_profile, \
             patch.object(PersonalizationService, 'get_page_history', new_callable=AsyncMock) as mock_get_page:

            mock_get_profile.return_value = mock_profile
            mock_get_page.return_value = mock_page_history

            # URL encode the page URL
            response = client.get("/personalization/history/%2Fdocs%2Fintro")

        assert response.status_code == 200
        data = response.json()
        assert data["found"] is True
        assert data["is_stale"] is False

    def test_returns_stale_when_profile_changed(self, client, mock_user, mock_profile):
        """Should indicate stale when profile has changed."""
        mock_time = datetime(2025, 1, 1, 12, 0, 0, tzinfo=timezone.utc)
        mock_page_history = PageHistoryResponse(
            found=True,
            profile_hash="old_hash" + "a" * 56,
            original_content_hash="b" * 64,
            created_at=mock_time,
            is_stale=True
        )

        with patch.object(ProfileService, 'get_profile', new_callable=AsyncMock) as mock_get_profile, \
             patch.object(PersonalizationService, 'get_page_history', new_callable=AsyncMock) as mock_get_page:

            mock_get_profile.return_value = mock_profile
            mock_get_page.return_value = mock_page_history

            response = client.get("/personalization/history/%2Fdocs%2Fintro")

        assert response.status_code == 200
        data = response.json()
        assert data["found"] is True
        assert data["is_stale"] is True

    def test_returns_not_found_when_no_history(self, client, mock_user, mock_profile):
        """Should indicate not found when page wasn't personalized."""
        mock_page_history = PageHistoryResponse(found=False)

        with patch.object(ProfileService, 'get_profile', new_callable=AsyncMock) as mock_get_profile, \
             patch.object(PersonalizationService, 'get_page_history', new_callable=AsyncMock) as mock_get_page:

            mock_get_profile.return_value = mock_profile
            mock_get_page.return_value = mock_page_history

            response = client.get("/personalization/history/%2Fdocs%2Fintro")

        assert response.status_code == 200
        data = response.json()
        assert data["found"] is False
