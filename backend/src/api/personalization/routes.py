# backend/src/api/personalization/routes.py
"""
Personalization API routes.

Endpoints:
- POST /personalization/personalize - Personalize page content
- GET /personalization/quota - Get quota status
- GET /personalization/history - Get personalization history
- GET /personalization/history/{pageUrl} - Check if page was personalized
"""

import logging
from typing import Annotated
from urllib.parse import unquote

from fastapi import APIRouter, Depends, HTTPException, status

from src.models.personalization import (
    PersonalizationRequest,
    PersonalizationResponse,
    QuotaStatus,
    PersonalizationHistoryResponse,
    PageHistoryResponse,
    QuotaExceededResponse,
    ErrorResponse,
)
from src.services.personalization_service import (
    PersonalizationService,
    compute_profile_hash,
    compute_content_hash,
    MIN_CONTENT_LENGTH,
)
from src.services.profile_service import ProfileService
from src.api.auth.dependencies import get_current_user, CurrentUser
from src.ai_agents.personalization_agent import (
    personalize_content,
    PersonalizationContext,
)

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/personalization", tags=["personalization"])


def get_personalization_service() -> PersonalizationService:
    """Get PersonalizationService instance."""
    return PersonalizationService()


def get_profile_service() -> ProfileService:
    """Get ProfileService instance."""
    return ProfileService()


@router.post(
    "/personalize",
    response_model=PersonalizationResponse,
    responses={
        400: {"model": ErrorResponse, "description": "Invalid request"},
        401: {"model": ErrorResponse, "description": "Unauthorized"},
        403: {"model": ErrorResponse, "description": "Profile incomplete"},
        429: {"model": QuotaExceededResponse, "description": "Quota exceeded"},
        500: {"model": ErrorResponse, "description": "Server error"},
        504: {"model": ErrorResponse, "description": "Timeout"},
    },
)
async def personalize_page_content(
    request: PersonalizationRequest,
    current_user: Annotated[CurrentUser, Depends(get_current_user)],
    personalization_service: Annotated[PersonalizationService, Depends(get_personalization_service)],
    profile_service: Annotated[ProfileService, Depends(get_profile_service)],
):
    """
    Personalize page content based on user's learning profile.

    - Requires valid JWT token
    - Requires completed user profile
    - Decrements daily quota (unless free re-personalization)
    - Returns personalized markdown content
    """
    user_id = current_user.id

    # Check if user has a profile
    profile = await profile_service.get_profile(user_id)
    if profile is None:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "error": "profile_incomplete",
                "message": "Please complete your profile before personalizing content",
                "profileWizardUrl": "/signup-wizard",
            },
        )

    # Check content length
    if len(request.page_content) < MIN_CONTENT_LENGTH:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={
                "error": "content_too_short",
                "message": f"Page content must be at least {MIN_CONTENT_LENGTH} characters",
            },
        )

    # Check if this is a free re-personalization
    is_free = request.is_free_repersonalization
    if is_free:
        # Verify the page was actually personalized before
        is_free = await personalization_service.is_free_repersonalization(
            user_id, request.page_url
        )

    # Check and decrement quota (unless free)
    quota_remaining = 0
    if not is_free:
        success, quota_remaining = await personalization_service.check_and_decrement_quota(user_id)
        if not success:
            quota_status = await personalization_service.get_quota_status(user_id)
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail={
                    "error": "quota_exceeded",
                    "message": f"Daily personalization limit reached ({quota_status.limit}/{quota_status.limit})",
                    "quotaStatus": quota_status.model_dump(),
                },
            )
    else:
        # Get current quota for response
        quota_status = await personalization_service.get_quota_status(user_id)
        quota_remaining = quota_status.remaining

    # Compute hashes
    profile_dict = profile.model_dump() if hasattr(profile, 'model_dump') else profile.__dict__
    profile_hash = compute_profile_hash(profile_dict)
    original_content_hash = compute_content_hash(request.page_content)

    # Build personalization context
    context = PersonalizationContext(
        user_id=user_id,
        tech_background=profile_dict.get('tech_background'),
        learning_mode=profile_dict.get('learning_mode'),
        learning_speed=profile_dict.get('learning_speed'),
        preferred_language=profile_dict.get('preferred_language', 'en'),
        education_level=profile_dict.get('education_level'),
        primary_goal=profile_dict.get('primary_goal'),
        focus_area=profile_dict.get('focus_area'),
    )

    # Personalize content
    try:
        personalized_content, processing_time_ms = await personalize_content(
            request.page_content,
            context,
        )
    except TimeoutError:
        raise HTTPException(
            status_code=status.HTTP_504_GATEWAY_TIMEOUT,
            detail={
                "error": "timeout",
                "message": "Personalization took too long. Please try again.",
            },
        )
    except Exception as e:
        logger.error(f"Personalization failed for user {user_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "personalization_failed",
                "message": "Failed to personalize content. Please try again.",
            },
        )

    # Record in history
    await personalization_service.upsert_history(
        user_id=user_id,
        page_url=request.page_url,
        profile_hash=profile_hash,
        original_content_hash=original_content_hash,
    )

    return PersonalizationResponse(
        personalized_content=personalized_content,
        profile_hash=profile_hash,
        original_content_hash=original_content_hash,
        processing_time_ms=processing_time_ms,
        quota_remaining=quota_remaining,
    )


@router.get("/quota", response_model=QuotaStatus)
async def get_quota_status(
    current_user: Annotated[CurrentUser, Depends(get_current_user)],
    personalization_service: Annotated[PersonalizationService, Depends(get_personalization_service)],
):
    """
    Get the current user's personalization quota status.

    - Requires valid JWT token
    - Returns limit, used, remaining, and reset time
    """
    return await personalization_service.get_quota_status(current_user.id)


@router.get("/history", response_model=PersonalizationHistoryResponse)
async def get_personalization_history(
    current_user: Annotated[CurrentUser, Depends(get_current_user)],
    personalization_service: Annotated[PersonalizationService, Depends(get_personalization_service)],
    profile_service: Annotated[ProfileService, Depends(get_profile_service)],
):
    """
    Get the user's personalization history.

    - Requires valid JWT token
    - Returns list of personalized pages with their profile hashes
    - Includes current profile hash for staleness detection
    """
    # Get current profile hash
    profile = await profile_service.get_profile(current_user.id)
    if profile is None:
        current_profile_hash = ""
    else:
        profile_dict = profile.model_dump() if hasattr(profile, 'model_dump') else profile.__dict__
        current_profile_hash = compute_profile_hash(profile_dict)

    return await personalization_service.get_history(current_user.id, current_profile_hash)


@router.get("/history/{page_url:path}", response_model=PageHistoryResponse)
async def check_page_personalized(
    page_url: str,
    current_user: Annotated[CurrentUser, Depends(get_current_user)],
    personalization_service: Annotated[PersonalizationService, Depends(get_personalization_service)],
    profile_service: Annotated[ProfileService, Depends(get_profile_service)],
):
    """
    Check if a specific page was previously personalized.

    - Requires valid JWT token
    - Returns found status, profile hash, and staleness indicator
    - page_url should be URL-encoded in the path
    """
    # Decode the URL
    decoded_url = unquote(page_url)

    # Get current profile hash
    profile = await profile_service.get_profile(current_user.id)
    if profile is None:
        current_profile_hash = ""
    else:
        profile_dict = profile.model_dump() if hasattr(profile, 'model_dump') else profile.__dict__
        current_profile_hash = compute_profile_hash(profile_dict)

    return await personalization_service.get_page_history(
        current_user.id,
        decoded_url,
        current_profile_hash,
    )
