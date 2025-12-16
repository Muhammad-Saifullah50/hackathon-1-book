# backend/src/services/translation_service.py
"""
Translation service for managing Urdu translation operations.

Handles:
- Content translation via AI agent
- Quota management (daily limits, checking, updating)
- History tracking (upsert records, check history)
- Content hashing for change detection
"""

import hashlib
import logging
from datetime import date, datetime, timezone, timedelta
from typing import Optional

from src.services.database import get_db_connection
from src.models.translation import (
    QuotaStatusResponse,
    TranslationHistoryResponse,
)
from src.ai_agents.translation_agent import translate_to_urdu

logger = logging.getLogger(__name__)

# Constants
DAILY_QUOTA_LIMIT = 5
MIN_CONTENT_LENGTH = 50  # Minimum characters for translation


def compute_content_hash(content: str) -> str:
    """
    Compute SHA-256 hash of content.

    Args:
        content: Page content string

    Returns:
        64-character hex string (SHA-256 hash)
    """
    return hashlib.sha256(content.encode()).hexdigest()


class TranslationService:
    """Service for translation operations."""

    def __init__(self):
        self.daily_limit = DAILY_QUOTA_LIMIT

    async def translate_content(
        self,
        content: str,
        page_url: str,
        user_id: Optional[str] = None,
        ip_hash: Optional[str] = None,
    ) -> tuple[str, int, str]:
        """
        Translate content to Urdu.

        Args:
            content: Original markdown content
            page_url: URL of the page being translated
            user_id: User ID (for authenticated users)
            ip_hash: Hashed IP (for anonymous users)

        Returns:
            Tuple of (translated_content, processing_time_ms, original_content_hash)

        Raises:
            ValueError: If content is too short
            Exception: If translation fails
        """
        if len(content) < MIN_CONTENT_LENGTH:
            raise ValueError(f"Content too short (minimum {MIN_CONTENT_LENGTH} characters)")

        # Compute content hash before translation
        original_content_hash = compute_content_hash(content)

        # Translate using AI agent
        translated_content, processing_time_ms = await translate_to_urdu(content)

        # Save to history
        await self.save_translation_history(
            user_id=user_id,
            ip_hash=ip_hash,
            page_url=page_url,
            original_content_hash=original_content_hash,
        )

        return translated_content, processing_time_ms, original_content_hash

    async def get_quota_status(
        self,
        user_id: Optional[str] = None,
        ip_hash: Optional[str] = None,
    ) -> QuotaStatusResponse:
        """
        Get the current quota status for a user or IP.

        Args:
            user_id: User ID (for authenticated users)
            ip_hash: Hashed IP (for anonymous users)

        Returns:
            QuotaStatusResponse with limit, used, remaining, and reset time
        """
        today = date.today()
        used_count = 0

        try:
            with get_db_connection() as conn:
                with conn.cursor() as cur:
                    if user_id:
                        cur.execute(
                            """
                            SELECT used_count FROM translation_quota
                            WHERE user_id = %s AND quota_date = %s
                            """,
                            (user_id, today)
                        )
                    else:
                        cur.execute(
                            """
                            SELECT used_count FROM translation_quota
                            WHERE ip_hash = %s AND quota_date = %s
                            """,
                            (ip_hash, today)
                        )
                    result = cur.fetchone()
                    if result:
                        used_count = result[0]
        except Exception as e:
            logger.error(f"Error fetching quota: {e}")
            # Return default quota on error

        # Calculate reset time (midnight UTC tomorrow)
        tomorrow = today + timedelta(days=1)
        resets_at = datetime.combine(tomorrow, datetime.min.time()).replace(tzinfo=timezone.utc)

        return QuotaStatusResponse(
            limit=self.daily_limit,
            used=used_count,
            remaining=max(0, self.daily_limit - used_count),
            resets_at=resets_at
        )

    async def check_and_update_quota(
        self,
        user_id: Optional[str] = None,
        ip_hash: Optional[str] = None,
    ) -> tuple[bool, int]:
        """
        Check if user/IP has quota and increment if available.

        Args:
            user_id: User ID (for authenticated users)
            ip_hash: Hashed IP (for anonymous users)

        Returns:
            Tuple of (success, remaining_quota)
        """
        today = date.today()

        try:
            with get_db_connection() as conn:
                with conn.cursor() as cur:
                    if user_id:
                        # Authenticated user quota
                        cur.execute(
                            """
                            INSERT INTO translation_quota (user_id, quota_date, used_count)
                            VALUES (%s, %s, 1)
                            ON CONFLICT (user_id, quota_date) DO UPDATE
                            SET used_count = translation_quota.used_count + 1,
                                updated_at = NOW()
                            WHERE translation_quota.used_count < %s
                            RETURNING used_count
                            """,
                            (user_id, today, self.daily_limit)
                        )
                    else:
                        # Anonymous user quota (IP-based)
                        cur.execute(
                            """
                            INSERT INTO translation_quota (ip_hash, quota_date, used_count)
                            VALUES (%s, %s, 1)
                            ON CONFLICT (ip_hash, quota_date) DO UPDATE
                            SET used_count = translation_quota.used_count + 1,
                                updated_at = NOW()
                            WHERE translation_quota.used_count < %s
                            RETURNING used_count
                            """,
                            (ip_hash, today, self.daily_limit)
                        )

                    result = cur.fetchone()

                    if result:
                        used_count = result[0]
                        remaining = max(0, self.daily_limit - used_count)
                        return True, remaining
                    else:
                        # Quota exhausted
                        return False, 0

        except Exception as e:
            logger.error(f"Error updating quota: {e}")
            raise

    async def save_translation_history(
        self,
        user_id: Optional[str],
        ip_hash: Optional[str],
        page_url: str,
        original_content_hash: str,
        target_language: str = "ur",
    ) -> None:
        """
        Save or update translation history record.

        Args:
            user_id: User ID (for authenticated users)
            ip_hash: Hashed IP (for anonymous users)
            page_url: Page URL path
            original_content_hash: SHA-256 hash of original content
            target_language: Target language code (default: "ur")
        """
        try:
            with get_db_connection() as conn:
                with conn.cursor() as cur:
                    if user_id:
                        cur.execute(
                            """
                            INSERT INTO translation_history
                                (user_id, page_url, target_language, original_content_hash)
                            VALUES (%s, %s, %s, %s)
                            ON CONFLICT (user_id, page_url, target_language)
                            DO UPDATE SET
                                original_content_hash = EXCLUDED.original_content_hash,
                                updated_at = NOW()
                            """,
                            (user_id, page_url, target_language, original_content_hash)
                        )
                    else:
                        cur.execute(
                            """
                            INSERT INTO translation_history
                                (ip_hash, page_url, target_language, original_content_hash)
                            VALUES (%s, %s, %s, %s)
                            ON CONFLICT (ip_hash, page_url, target_language)
                            DO UPDATE SET
                                original_content_hash = EXCLUDED.original_content_hash,
                                updated_at = NOW()
                            """,
                            (ip_hash, page_url, target_language, original_content_hash)
                        )
        except Exception as e:
            logger.error(f"Error saving translation history: {e}")
            raise

    async def get_translation_history(
        self,
        page_url: str,
        user_id: Optional[str] = None,
        ip_hash: Optional[str] = None,
        target_language: str = "ur",
        current_content_hash: Optional[str] = None,
    ) -> TranslationHistoryResponse:
        """
        Check if a page was previously translated.

        Args:
            page_url: Page URL to check
            user_id: User ID (for authenticated users)
            ip_hash: Hashed IP (for anonymous users)
            target_language: Target language code
            current_content_hash: Hash of current content for staleness detection

        Returns:
            TranslationHistoryResponse with translation metadata
        """
        try:
            with get_db_connection() as conn:
                with conn.cursor() as cur:
                    if user_id:
                        cur.execute(
                            """
                            SELECT original_content_hash, created_at
                            FROM translation_history
                            WHERE user_id = %s AND page_url = %s AND target_language = %s
                            """,
                            (user_id, page_url, target_language)
                        )
                    else:
                        cur.execute(
                            """
                            SELECT original_content_hash, created_at
                            FROM translation_history
                            WHERE ip_hash = %s AND page_url = %s AND target_language = %s
                            """,
                            (ip_hash, page_url, target_language)
                        )

                    result = cur.fetchone()

                    if result:
                        stored_hash = result[0]
                        translated_at = result[1]
                        content_changed = (
                            current_content_hash is not None and
                            stored_hash != current_content_hash
                        )

                        return TranslationHistoryResponse(
                            has_translation=True,
                            original_content_hash=stored_hash,
                            translated_at=translated_at,
                            content_changed=content_changed,
                        )
                    else:
                        return TranslationHistoryResponse(
                            has_translation=False,
                            content_changed=False,
                        )

        except Exception as e:
            logger.error(f"Error checking translation history: {e}")
            raise
