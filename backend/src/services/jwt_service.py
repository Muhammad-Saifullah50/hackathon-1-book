"""
JWT validation service for Better Auth tokens.

Validates JWT tokens issued by Better Auth using:
- JWKS (JSON Web Key Set) for public key retrieval
- RS256 algorithm verification
- Issuer and audience validation
- Token expiration checking
- LRU caching for JWKS to reduce latency
"""

import os
import logging
from functools import lru_cache
from typing import Optional, Dict, Any
from dataclasses import dataclass

import jwt
from jwt import PyJWKClient, InvalidTokenError, ExpiredSignatureError

logger = logging.getLogger(__name__)


@dataclass
class TokenPayload:
    """Parsed and validated JWT token payload."""

    user_id: str
    email: Optional[str]
    issued_at: int
    expires_at: int
    issuer: str
    audience: str
    raw: Dict[str, Any]


class JWTValidationError(Exception):
    """Base exception for JWT validation errors."""

    pass


class TokenExpiredError(JWTValidationError):
    """Token has expired."""

    pass


class InvalidTokenError(JWTValidationError):
    """Token is invalid or tampered."""

    pass


class MissingTokenError(JWTValidationError):
    """No token provided."""

    pass


def get_jwks_url() -> str:
    """Get JWKS URL from environment."""
    url = os.environ.get("JWKS_URL")
    if not url:
        raise ValueError("JWKS_URL environment variable is not set")
    return url


def get_jwt_issuer() -> str:
    """Get expected JWT issuer from environment."""
    return os.environ.get("JWT_ISSUER", "http://localhost:3001")


def get_jwt_audience() -> str:
    """Get expected JWT audience from environment."""
    return os.environ.get("JWT_AUDIENCE", "http://localhost:3001")


@lru_cache(maxsize=1)
def get_jwks_client() -> PyJWKClient:
    """
    Get cached JWKS client.

    Uses LRU cache to avoid creating new clients for each request.
    The client itself caches the JWKS keys internally.

    Returns:
        PyJWKClient instance
    """
    jwks_url = get_jwks_url()
    logger.info(f"Initializing JWKS client with URL: {jwks_url}")
    return PyJWKClient(jwks_url, cache_keys=True, lifespan=3600)  # Cache keys for 1 hour


def validate_jwt(token: str) -> TokenPayload:
    """
    Validate a JWT token from Better Auth.

    Args:
        token: JWT token string (without 'Bearer ' prefix)

    Returns:
        TokenPayload with validated claims

    Raises:
        MissingTokenError: If token is empty or None
        TokenExpiredError: If token has expired
        InvalidTokenError: If token is invalid, tampered, or verification fails
    """
    if not token:
        raise MissingTokenError("No token provided")

    # Remove 'Bearer ' prefix if present
    if token.startswith("Bearer "):
        token = token[7:]

    try:
        # Get the signing key from JWKS
        jwks_client = get_jwks_client()
        signing_key = jwks_client.get_signing_key_from_jwt(token)

        # Decode and validate the token
        payload = jwt.decode(
            token,
            signing_key.key,
            algorithms=["RS256"],
            audience=get_jwt_audience(),
            issuer=get_jwt_issuer(),
            options={
                "verify_signature": True,
                "verify_exp": True,
                "verify_iat": True,
                "verify_aud": True,
                "verify_iss": True,
                "require": ["exp", "iat", "sub"],
            },
        )

        # Extract user information
        user_id = payload.get("sub")
        if not user_id:
            raise InvalidTokenError("Token missing 'sub' claim (user ID)")

        return TokenPayload(
            user_id=user_id,
            email=payload.get("email"),
            issued_at=payload.get("iat", 0),
            expires_at=payload.get("exp", 0),
            issuer=payload.get("iss", ""),
            audience=payload.get("aud", ""),
            raw=payload,
        )

    except ExpiredSignatureError:
        logger.warning("JWT token has expired")
        raise TokenExpiredError("Token has expired")

    except jwt.InvalidAudienceError:
        logger.warning("JWT token has invalid audience")
        raise InvalidTokenError("Token has invalid audience")

    except jwt.InvalidIssuerError:
        logger.warning("JWT token has invalid issuer")
        raise InvalidTokenError("Token has invalid issuer")

    except jwt.DecodeError as e:
        logger.warning(f"JWT decode error: {e}")
        raise InvalidTokenError(f"Token decode failed: {e}")

    except Exception as e:
        logger.error(f"JWT validation error: {e}")
        raise InvalidTokenError(f"Token validation failed: {e}")


def extract_token_from_header(authorization_header: Optional[str]) -> str:
    """
    Extract JWT token from Authorization header.

    Args:
        authorization_header: Value of Authorization header

    Returns:
        Token string without 'Bearer ' prefix

    Raises:
        MissingTokenError: If header is missing or malformed
    """
    if not authorization_header:
        raise MissingTokenError("Authorization header is missing")

    if not authorization_header.startswith("Bearer "):
        raise InvalidTokenError("Authorization header must start with 'Bearer '")

    token = authorization_header[7:].strip()
    if not token:
        raise MissingTokenError("Token is empty")

    return token


def clear_jwks_cache() -> None:
    """Clear the JWKS client cache (useful for testing or key rotation)."""
    get_jwks_client.cache_clear()
    logger.info("JWKS client cache cleared")
