# backend/src/services/personalization_service.py
"""
Personalization service for managing content personalization.

Handles:
- Quota management (daily limits, checking, decrementing)
- History tracking (upsert records, fetch history)
- Profile hashing for change detection
- Content hashing for source change detection
"""

import hashlib
import json
import logging
from datetime import date, datetime, timezone, timedelta
from typing import Optional

from src.services.database import get_db_connection
from src.models.personalization import (
    QuotaStatus,
    PersonalizationHistoryItem,
    PersonalizationHistoryResponse,
    PageHistoryResponse,
)

logger = logging.getLogger(__name__)

# Constants
DAILY_QUOTA_LIMIT = 5
MIN_CONTENT_LENGTH = 100  # Minimum characters for personalization


def compute_profile_hash(profile: dict) -> str:
    """
    Compute SHA-256 hash of profile attributes.
    Only includes personalization-relevant fields.

    Args:
        profile: User profile dictionary

    Returns:
        64-character hex string (SHA-256 hash)
    """
    relevant_fields = {
        'tech_background': profile.get('tech_background'),
        'learning_mode': profile.get('learning_mode'),
        'learning_speed': profile.get('learning_speed'),
        'preferred_language': profile.get('preferred_language', 'en'),
        'education_level': profile.get('education_level'),
        'primary_goal': profile.get('primary_goal'),
        'focus_area': profile.get('focus_area'),
    }
    # Sort keys for deterministic serialization
    serialized = json.dumps(relevant_fields, sort_keys=True)
    return hashlib.sha256(serialized.encode()).hexdigest()


def compute_content_hash(content: str) -> str:
    """
    Compute SHA-256 hash of content.

    Args:
        content: Page content string

    Returns:
        64-character hex string (SHA-256 hash)
    """
    return hashlib.sha256(content.encode()).hexdigest()


class PersonalizationService:
    """Service for personalization operations."""

    def __init__(self):
        self.daily_limit = DAILY_QUOTA_LIMIT

    async def get_quota_status(self, user_id: str) -> QuotaStatus:
        """
        Get the current quota status for a user.

        Args:
            user_id: User's ID

        Returns:
            QuotaStatus with limit, used, remaining, and reset time
        """
        today = date.today()
        used_count = 0

        try:
            with get_db_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute(
                        """
                        SELECT used_count FROM personalization_quota
                        WHERE user_id = %s AND date = %s
                        """,
                        (user_id, today)
                    )
                    result = cur.fetchone()
                    if result:
                        used_count = result[0]
        except Exception as e:
            logger.error(f"Error fetching quota for user {user_id}: {e}")
            # Return default quota on error

        # Calculate reset time (midnight UTC tomorrow)
        tomorrow = today + timedelta(days=1)
        resets_at = datetime.combine(tomorrow, datetime.min.time()).replace(tzinfo=timezone.utc)

        return QuotaStatus(
            limit=self.daily_limit,
            used=used_count,
            remaining=max(0, self.daily_limit - used_count),
            resets_at=resets_at
        )

    async def check_and_decrement_quota(self, user_id: str) -> tuple[bool, int]:
        """
        Check if user has quota and decrement if available.

        Args:
            user_id: User's ID

        Returns:
            Tuple of (success, remaining_quota)
        """
        today = date.today()

        try:
            with get_db_connection() as conn:
                with conn.cursor() as cur:
                    # Upsert quota record and increment if under limit
                    cur.execute(
                        """
                        INSERT INTO personalization_quota (user_id, date, used_count)
                        VALUES (%s, %s, 1)
                        ON CONFLICT (user_id, date) DO UPDATE
                        SET used_count = personalization_quota.used_count + 1,
                            updated_at = NOW()
                        WHERE personalization_quota.used_count < %s
                        RETURNING used_count
                        """,
                        (user_id, today, self.daily_limit)
                    )
                    result = cur.fetchone()

                    if result:
                        used_count = result[0]
                        remaining = max(0, self.daily_limit - used_count)
                        return True, remaining
                    else:
                        # Quota exhausted - fetch current count
                        cur.execute(
                            """
                            SELECT used_count FROM personalization_quota
                            WHERE user_id = %s AND date = %s
                            """,
                            (user_id, today)
                        )
                        count_result = cur.fetchone()
                        used_count = count_result[0] if count_result else self.daily_limit
                        return False, 0

        except Exception as e:
            logger.error(f"Error updating quota for user {user_id}: {e}")
            raise

    async def upsert_history(
        self,
        user_id: str,
        page_url: str,
        profile_hash: str,
        original_content_hash: str
    ) -> None:
        """
        Create or update personalization history record.

        Args:
            user_id: User's ID
            page_url: Page URL path
            profile_hash: SHA-256 hash of profile
            original_content_hash: SHA-256 hash of original content
        """
        try:
            with get_db_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute(
                        """
                        INSERT INTO personalization_history
                            (user_id, page_url, profile_hash, original_content_hash)
                        VALUES (%s, %s, %s, %s)
                        ON CONFLICT (user_id, page_url)
                        DO UPDATE SET
                            profile_hash = EXCLUDED.profile_hash,
                            original_content_hash = EXCLUDED.original_content_hash,
                            updated_at = NOW()
                        """,
                        (user_id, page_url, profile_hash, original_content_hash)
                    )
        except Exception as e:
            logger.error(f"Error upserting history for user {user_id}, page {page_url}: {e}")
            raise

    async def get_history(self, user_id: str, current_profile_hash: str) -> PersonalizationHistoryResponse:
        """
        Get user's personalization history.

        Args:
            user_id: User's ID
            current_profile_hash: Current profile hash for staleness detection

        Returns:
            PersonalizationHistoryResponse with pages and current profile hash
        """
        pages = []

        try:
            with get_db_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute(
                        """
                        SELECT page_url, profile_hash, created_at
                        FROM personalization_history
                        WHERE user_id = %s
                        ORDER BY updated_at DESC
                        """,
                        (user_id,)
                    )
                    results = cur.fetchall()

                    for row in results:
                        pages.append(PersonalizationHistoryItem(
                            url=row[0],
                            profile_hash=row[1],
                            created_at=row[2]
                        ))
        except Exception as e:
            logger.error(f"Error fetching history for user {user_id}: {e}")
            raise

        return PersonalizationHistoryResponse(
            pages=pages,
            current_profile_hash=current_profile_hash
        )

    async def get_page_history(
        self,
        user_id: str,
        page_url: str,
        current_profile_hash: str
    ) -> PageHistoryResponse:
        """
        Check if a specific page was personalized.

        Args:
            user_id: User's ID
            page_url: Page URL to check
            current_profile_hash: Current profile hash for staleness detection

        Returns:
            PageHistoryResponse with found status and metadata
        """
        try:
            with get_db_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute(
                        """
                        SELECT profile_hash, original_content_hash, created_at
                        FROM personalization_history
                        WHERE user_id = %s AND page_url = %s
                        """,
                        (user_id, page_url)
                    )
                    result = cur.fetchone()

                    if result:
                        stored_profile_hash = result[0]
                        is_stale = stored_profile_hash != current_profile_hash

                        return PageHistoryResponse(
                            found=True,
                            profile_hash=stored_profile_hash,
                            original_content_hash=result[1],
                            created_at=result[2],
                            is_stale=is_stale
                        )
                    else:
                        return PageHistoryResponse(found=False)

        except Exception as e:
            logger.error(f"Error checking page history for user {user_id}, page {page_url}: {e}")
            raise

    async def is_free_repersonalization(
        self,
        user_id: str,
        page_url: str
    ) -> bool:
        """
        Check if this is a free re-personalization (page was previously personalized).

        Args:
            user_id: User's ID
            page_url: Page URL to check

        Returns:
            True if page was previously personalized (free re-personalization)
        """
        try:
            with get_db_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute(
                        """
                        SELECT 1 FROM personalization_history
                        WHERE user_id = %s AND page_url = %s
                        LIMIT 1
                        """,
                        (user_id, page_url)
                    )
                    return cur.fetchone() is not None
        except Exception as e:
            logger.error(f"Error checking free repersonalization for user {user_id}: {e}")
            return False
