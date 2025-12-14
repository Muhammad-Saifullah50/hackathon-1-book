"""
FastAPI authentication dependencies.

Provides dependency injection for JWT-authenticated endpoints:
- get_current_user: Requires valid JWT, returns user info
- get_optional_user: Allows anonymous access, returns user or None
"""

import logging
from typing import Optional
from dataclasses import dataclass

from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials

from src.services.jwt_service import (
    validate_jwt,
    TokenPayload,
    MissingTokenError,
    TokenExpiredError,
    InvalidTokenError,
)

logger = logging.getLogger(__name__)

# HTTP Bearer token security scheme
bearer_scheme = HTTPBearer(auto_error=False)


@dataclass
class CurrentUser:
    """Authenticated user information extracted from JWT."""

    id: str
    email: Optional[str]

    @classmethod
    def from_token_payload(cls, payload: TokenPayload) -> "CurrentUser":
        """Create CurrentUser from validated token payload."""
        return cls(id=payload.user_id, email=payload.email)


async def get_current_user(
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(bearer_scheme),
) -> CurrentUser:
    """
    FastAPI dependency that validates JWT and returns current user.

    Usage:
        @app.get("/protected")
        async def protected_route(user: CurrentUser = Depends(get_current_user)):
            return {"user_id": user.id}

    Args:
        credentials: HTTP Bearer credentials from request

    Returns:
        CurrentUser with user ID and email

    Raises:
        HTTPException 401: If token is missing, expired, or invalid
    """
    if credentials is None:
        logger.warning("No credentials provided for protected endpoint")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required",
            headers={"WWW-Authenticate": "Bearer"},
        )

    try:
        token_payload = validate_jwt(credentials.credentials)
        return CurrentUser.from_token_payload(token_payload)

    except MissingTokenError as e:
        logger.warning(f"Missing token: {e}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required",
            headers={"WWW-Authenticate": "Bearer"},
        )

    except TokenExpiredError:
        logger.warning("Expired token used")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Token has expired",
            headers={"WWW-Authenticate": "Bearer"},
        )

    except InvalidTokenError as e:
        logger.warning(f"Invalid token: {e}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid authentication token",
            headers={"WWW-Authenticate": "Bearer"},
        )


async def get_optional_user(
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(bearer_scheme),
) -> Optional[CurrentUser]:
    """
    FastAPI dependency that optionally validates JWT.

    Returns user if valid token provided, None otherwise.
    Does not raise exceptions for missing/invalid tokens.

    Usage:
        @app.get("/public")
        async def public_route(user: Optional[CurrentUser] = Depends(get_optional_user)):
            if user:
                return {"user_id": user.id}
            return {"message": "Anonymous access"}

    Args:
        credentials: HTTP Bearer credentials from request

    Returns:
        CurrentUser if valid token, None otherwise
    """
    if credentials is None:
        return None

    try:
        token_payload = validate_jwt(credentials.credentials)
        return CurrentUser.from_token_payload(token_payload)
    except Exception as e:
        logger.debug(f"Optional auth failed: {e}")
        return None
