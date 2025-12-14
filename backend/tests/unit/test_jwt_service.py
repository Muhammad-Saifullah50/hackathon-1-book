"""
Unit tests for JWT validation service.

Tests:
- Token validation with valid JWT
- Token expiration handling
- Invalid token rejection
- Missing token handling
- JWKS caching
"""

import os
import pytest
from unittest.mock import patch, MagicMock
from datetime import datetime, timedelta

# These tests will FAIL until implementation is complete (TDD approach)


class TestJWTValidation:
    """Tests for JWT validation service."""

    def test_validate_jwt_with_valid_token(self):
        """Test that valid JWT is successfully validated."""
        from src.services.jwt_service import validate_jwt, TokenPayload, clear_jwks_cache

        # Mock the JWKS client and JWT decode
        with patch.dict(
            os.environ,
            {
                "JWKS_URL": "http://localhost:3001/api/auth/jwks",
                "JWT_ISSUER": "http://localhost:3001",
                "JWT_AUDIENCE": "http://localhost:3001",
            },
        ):
            with patch("src.services.jwt_service.PyJWKClient") as mock_client_class:
                with patch("src.services.jwt_service.jwt.decode") as mock_decode:
                    mock_client = MagicMock()
                    mock_key = MagicMock()
                    mock_key.key = "test-key"
                    mock_client.get_signing_key_from_jwt.return_value = mock_key
                    mock_client_class.return_value = mock_client

                    mock_decode.return_value = {
                        "sub": "user-123",
                        "email": "test@example.com",
                        "iat": 1699999999,
                        "exp": 1700604799,
                        "iss": "http://localhost:3001",
                        "aud": "http://localhost:3001",
                    }

                    clear_jwks_cache()
                    result = validate_jwt("valid-token")

                    assert isinstance(result, TokenPayload)
                    assert result.user_id == "user-123"
                    assert result.email == "test@example.com"

    def test_validate_jwt_raises_on_expired_token(self):
        """Test that expired token raises TokenExpiredError."""
        from src.services.jwt_service import validate_jwt, TokenExpiredError, clear_jwks_cache
        import jwt

        with patch.dict(
            os.environ,
            {
                "JWKS_URL": "http://localhost:3001/api/auth/jwks",
                "JWT_ISSUER": "http://localhost:3001",
                "JWT_AUDIENCE": "http://localhost:3001",
            },
        ):
            with patch("src.services.jwt_service.PyJWKClient") as mock_client_class:
                with patch("src.services.jwt_service.jwt.decode") as mock_decode:
                    mock_client = MagicMock()
                    mock_key = MagicMock()
                    mock_key.key = "test-key"
                    mock_client.get_signing_key_from_jwt.return_value = mock_key
                    mock_client_class.return_value = mock_client

                    mock_decode.side_effect = jwt.ExpiredSignatureError("Token expired")

                    clear_jwks_cache()
                    with pytest.raises(TokenExpiredError):
                        validate_jwt("expired-token")

    def test_validate_jwt_raises_on_invalid_token(self):
        """Test that invalid token raises InvalidTokenError."""
        from src.services.jwt_service import validate_jwt, InvalidTokenError, clear_jwks_cache
        import jwt

        with patch.dict(
            os.environ,
            {
                "JWKS_URL": "http://localhost:3001/api/auth/jwks",
                "JWT_ISSUER": "http://localhost:3001",
                "JWT_AUDIENCE": "http://localhost:3001",
            },
        ):
            with patch("src.services.jwt_service.PyJWKClient") as mock_client_class:
                with patch("src.services.jwt_service.jwt.decode") as mock_decode:
                    mock_client = MagicMock()
                    mock_key = MagicMock()
                    mock_key.key = "test-key"
                    mock_client.get_signing_key_from_jwt.return_value = mock_key
                    mock_client_class.return_value = mock_client

                    mock_decode.side_effect = jwt.DecodeError("Invalid token")

                    clear_jwks_cache()
                    with pytest.raises(InvalidTokenError):
                        validate_jwt("invalid-token")

    def test_validate_jwt_raises_on_missing_token(self):
        """Test that empty token raises MissingTokenError."""
        from src.services.jwt_service import validate_jwt, MissingTokenError

        with pytest.raises(MissingTokenError):
            validate_jwt("")

        with pytest.raises(MissingTokenError):
            validate_jwt(None)

    def test_extract_token_from_header_valid(self):
        """Test token extraction from valid Authorization header."""
        from src.services.jwt_service import extract_token_from_header

        token = extract_token_from_header("Bearer abc123")
        assert token == "abc123"

    def test_extract_token_from_header_missing(self):
        """Test that missing header raises MissingTokenError."""
        from src.services.jwt_service import extract_token_from_header, MissingTokenError

        with pytest.raises(MissingTokenError):
            extract_token_from_header(None)

        with pytest.raises(MissingTokenError):
            extract_token_from_header("")

    def test_extract_token_from_header_invalid_format(self):
        """Test that invalid header format raises InvalidTokenError."""
        from src.services.jwt_service import extract_token_from_header, InvalidTokenError

        with pytest.raises(InvalidTokenError):
            extract_token_from_header("Basic abc123")

        with pytest.raises(InvalidTokenError):
            extract_token_from_header("abc123")

    def test_validate_jwt_strips_bearer_prefix(self):
        """Test that Bearer prefix is stripped from token."""
        from src.services.jwt_service import validate_jwt, clear_jwks_cache

        with patch.dict(
            os.environ,
            {
                "JWKS_URL": "http://localhost:3001/api/auth/jwks",
                "JWT_ISSUER": "http://localhost:3001",
                "JWT_AUDIENCE": "http://localhost:3001",
            },
        ):
            with patch("src.services.jwt_service.PyJWKClient") as mock_client_class:
                with patch("src.services.jwt_service.jwt.decode") as mock_decode:
                    mock_client = MagicMock()
                    mock_key = MagicMock()
                    mock_key.key = "test-key"
                    mock_client.get_signing_key_from_jwt.return_value = mock_key
                    mock_client_class.return_value = mock_client

                    mock_decode.return_value = {
                        "sub": "user-123",
                        "email": "test@example.com",
                        "iat": 1699999999,
                        "exp": 1700604799,
                        "iss": "http://localhost:3001",
                        "aud": "http://localhost:3001",
                    }

                    clear_jwks_cache()
                    result = validate_jwt("Bearer valid-token")

                    # Verify the token was passed without Bearer prefix
                    mock_client.get_signing_key_from_jwt.assert_called_with("valid-token")
