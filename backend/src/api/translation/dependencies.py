# backend/src/api/translation/dependencies.py
"""
Dependencies for translation API endpoints.

Provides:
- IP extraction helper for anonymous user quota tracking
- Optional authentication dependency
"""

import hashlib
from typing import Optional

from fastapi import Request, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials

from src.services.jwt_service import validate_jwt


# Optional bearer token authentication
optional_bearer = HTTPBearer(auto_error=False)


def get_client_ip(request: Request) -> str:
    """
    Extract client IP address from request, handling proxy chains.

    Args:
        request: FastAPI request object

    Returns:
        Client IP address string
    """
    # Check X-Forwarded-For first (common with proxies/load balancers)
    forwarded = request.headers.get("X-Forwarded-For")
    if forwarded:
        # First IP in chain is the original client
        client_ip = forwarded.split(",")[0].strip()
    else:
        # Direct connection
        client_ip = request.client.host if request.client else "unknown"
    return client_ip


def hash_ip(ip: str) -> str:
    """
    Hash IP address for privacy-preserving storage.

    Args:
        ip: Raw IP address string

    Returns:
        32-character truncated SHA-256 hash
    """
    return hashlib.sha256(ip.encode()).hexdigest()[:32]


async def get_optional_user_id(
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(optional_bearer)
) -> Optional[str]:
    """
    Get user ID from optional bearer token.

    Returns None if no token provided or token is invalid.
    This allows anonymous users to use translation without authentication.

    Args:
        credentials: Optional HTTP bearer credentials

    Returns:
        User ID if authenticated, None otherwise
    """
    if not credentials:
        return None

    try:
        payload = validate_jwt(credentials.credentials)
        return payload.user_id
    except Exception:
        return None


def get_user_or_ip_hash(
    request: Request,
    user_id: Optional[str] = Depends(get_optional_user_id)
) -> tuple[Optional[str], Optional[str]]:
    """
    Get either user_id (for authenticated users) or ip_hash (for anonymous users).

    Args:
        request: FastAPI request object
        user_id: Optional authenticated user ID

    Returns:
        Tuple of (user_id, ip_hash) where exactly one is not None
    """
    if user_id:
        return (user_id, None)
    else:
        ip = get_client_ip(request)
        return (None, hash_ip(ip))
