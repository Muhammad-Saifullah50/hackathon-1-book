"""
Contract tests for authentication middleware.

Tests:
- Protected endpoint returns 401 without token
- Protected endpoint returns 401 with expired token
- Protected endpoint returns 401 with invalid token
- Protected endpoint returns 200 with valid token
"""

import pytest
from fastapi import FastAPI, Depends
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock

# These tests will FAIL until implementation is complete (TDD approach)


class TestAuthMiddleware:
    """Contract tests for JWT authentication middleware."""

    @pytest.fixture
    def app_with_protected_route(self):
        """Create a test app with a protected route."""
        from src.api.auth.dependencies import get_current_user, CurrentUser

        app = FastAPI()

        @app.get("/protected")
        async def protected_route(user: CurrentUser = Depends(get_current_user)):
            return {"user_id": user.id, "email": user.email}

        return app

    @pytest.fixture
    def client(self, app_with_protected_route):
        """Create test client."""
        return TestClient(app_with_protected_route)

    def test_protected_endpoint_returns_401_without_token(self, client):
        """Test that protected endpoint returns 401 when no token is provided."""
        response = client.get("/protected")
        assert response.status_code == 401
        assert "detail" in response.json()

    def test_protected_endpoint_returns_401_with_expired_token(self, client):
        """Test that protected endpoint returns 401 with expired token."""
        from src.services.jwt_service import TokenExpiredError

        with patch("src.api.auth.dependencies.validate_jwt") as mock_validate:
            mock_validate.side_effect = TokenExpiredError("Token expired")

            response = client.get(
                "/protected", headers={"Authorization": "Bearer expired-token"}
            )

            assert response.status_code == 401
            assert "expired" in response.json()["detail"].lower()

    def test_protected_endpoint_returns_401_with_invalid_token(self, client):
        """Test that protected endpoint returns 401 with invalid token."""
        from src.services.jwt_service import InvalidTokenError

        with patch("src.api.auth.dependencies.validate_jwt") as mock_validate:
            mock_validate.side_effect = InvalidTokenError("Invalid token")

            response = client.get(
                "/protected", headers={"Authorization": "Bearer invalid-token"}
            )

            assert response.status_code == 401
            assert "invalid" in response.json()["detail"].lower()

    def test_protected_endpoint_returns_200_with_valid_token(self, client):
        """Test that protected endpoint returns 200 with valid token."""
        from src.services.jwt_service import TokenPayload

        with patch("src.api.auth.dependencies.validate_jwt") as mock_validate:
            mock_validate.return_value = TokenPayload(
                user_id="user-123",
                email="test@example.com",
                issued_at=1699999999,
                expires_at=1700604799,
                issuer="http://localhost:3001",
                audience="http://localhost:3001",
                raw={},
            )

            response = client.get(
                "/protected", headers={"Authorization": "Bearer valid-token"}
            )

            assert response.status_code == 200
            data = response.json()
            assert data["user_id"] == "user-123"
            assert data["email"] == "test@example.com"

    def test_www_authenticate_header_present_on_401(self, client):
        """Test that WWW-Authenticate header is present on 401 response."""
        response = client.get("/protected")
        assert response.status_code == 401
        assert "WWW-Authenticate" in response.headers
        assert response.headers["WWW-Authenticate"] == "Bearer"
