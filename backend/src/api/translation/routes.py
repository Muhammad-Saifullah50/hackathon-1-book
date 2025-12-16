# backend/src/api/translation/routes.py
"""
Translation API routes for Urdu translation feature.

Endpoints:
- POST /api/translate/urdu - Translate content to Urdu
- GET /api/translate/quota - Get quota status
- GET /api/translate/history - Check translation history
"""

import logging
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, Request

from src.models.translation import (
    TranslationRequest,
    TranslationResponse,
    QuotaStatusResponse,
    TranslationHistoryResponse,
    ErrorResponse,
)
from src.services.translation_service import TranslationService
from src.api.translation.dependencies import get_user_or_ip_hash

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/translate", tags=["translation"])

# Singleton service instance
translation_service = TranslationService()


@router.post(
    "/urdu",
    response_model=TranslationResponse,
    responses={
        400: {"model": ErrorResponse, "description": "Invalid request"},
        429: {"model": ErrorResponse, "description": "Quota exceeded"},
        500: {"model": ErrorResponse, "description": "Translation service error"},
    },
)
async def translate_to_urdu(
    request: TranslationRequest,
    user_or_ip: tuple[Optional[str], Optional[str]] = Depends(get_user_or_ip_hash),
):
    """
    Translate page content from English to Urdu.

    Preserves code blocks, images, and markdown structure.
    Requires quota availability (5 translations/day per user or IP).
    """
    user_id, ip_hash = user_or_ip

    # Check and update quota
    try:
        has_quota, remaining = await translation_service.check_and_update_quota(
            user_id=user_id,
            ip_hash=ip_hash,
        )

        if not has_quota:
            quota_status = await translation_service.get_quota_status(
                user_id=user_id,
                ip_hash=ip_hash,
            )
            raise HTTPException(
                status_code=429,
                detail={
                    "error": "Daily translation limit reached",
                    "code": "QUOTA_EXCEEDED",
                    "details": {
                        "used": quota_status.used,
                        "limit": quota_status.limit,
                        "resets_at": quota_status.resets_at.isoformat(),
                    },
                },
            )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Quota check failed: {e}")
        raise HTTPException(status_code=500, detail={"error": str(e), "code": "SERVICE_UNAVAILABLE"})

    # Perform translation
    try:
        translated_content, processing_time_ms, original_content_hash = await translation_service.translate_content(
            content=request.page_content,
            page_url=request.page_url,
            user_id=user_id,
            ip_hash=ip_hash,
        )

        # Get updated quota status
        quota_status = await translation_service.get_quota_status(
            user_id=user_id,
            ip_hash=ip_hash,
        )

        return TranslationResponse(
            translated_content=translated_content,
            source_language=request.source_language,
            target_language=request.target_language,
            processing_time_ms=processing_time_ms,
            original_content_hash=original_content_hash,
            quota_remaining=quota_status.remaining,
            quota_limit=quota_status.limit,
        )

    except ValueError as e:
        raise HTTPException(
            status_code=400,
            detail={"error": str(e), "code": "INVALID_CONTENT"},
        )
    except Exception as e:
        logger.error(f"Translation failed: {e}")
        raise HTTPException(
            status_code=500,
            detail={"error": "Translation service error", "code": "TRANSLATION_FAILED"},
        )


@router.get("/quota", response_model=QuotaStatusResponse)
async def get_quota(
    user_or_ip: tuple[Optional[str], Optional[str]] = Depends(get_user_or_ip_hash),
):
    """
    Get current translation quota status.

    Returns the number of translations used today and remaining quota.
    """
    user_id, ip_hash = user_or_ip

    try:
        quota_status = await translation_service.get_quota_status(
            user_id=user_id,
            ip_hash=ip_hash,
        )
        return quota_status
    except Exception as e:
        logger.error(f"Failed to get quota status: {e}")
        raise HTTPException(status_code=500, detail={"error": str(e), "code": "SERVICE_UNAVAILABLE"})


@router.get("/history", response_model=TranslationHistoryResponse)
async def check_history(
    page_url: str,
    target_language: str = "ur",
    current_content_hash: Optional[str] = None,
    user_or_ip: tuple[Optional[str], Optional[str]] = Depends(get_user_or_ip_hash),
):
    """
    Check if a page was previously translated.

    Used for cross-device awareness and detecting stale content.
    """
    user_id, ip_hash = user_or_ip

    try:
        history = await translation_service.get_translation_history(
            page_url=page_url,
            user_id=user_id,
            ip_hash=ip_hash,
            target_language=target_language,
            current_content_hash=current_content_hash,
        )
        return history
    except Exception as e:
        logger.error(f"Failed to check translation history: {e}")
        raise HTTPException(status_code=500, detail={"error": str(e), "code": "SERVICE_UNAVAILABLE"})
