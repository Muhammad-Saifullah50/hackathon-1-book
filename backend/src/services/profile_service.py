# backend/src/services/profile_service.py
"""
Profile service for user profile CRUD operations.

Uses Neon PostgreSQL (via psycopg2) for data persistence.
Replaces previous Supabase implementation.
"""

import logging
from typing import Optional, Dict, Any
from pydantic import ValidationError
from src.models.profile import UserProfile
from src.services.database import get_db_connection

logger = logging.getLogger(__name__)


class ProfileService:
    """Service for managing user profiles in Neon database."""

    async def create_profile(self, user_id: str, profile_data: Dict[str, Any]) -> Optional[UserProfile]:
        """
        Create a new user profile.

        Args:
            user_id: User ID from Better Auth
            profile_data: Profile fields to set

        Returns:
            Created UserProfile or None if creation fails
        """
        try:
            # Ensure user_id is included in the profile data for validation
            profile_data["user_id"] = str(user_id)

            # Validate input data against Pydantic model
            profile_to_create = UserProfile(**profile_data)

            with get_db_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute(
                        """
                        INSERT INTO user_profile (
                            user_id, age_range, education_level, tech_background,
                            focus_area, primary_goal, learning_mode, learning_speed,
                            time_per_week, preferred_language
                        )
                        VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
                        RETURNING id, user_id, age_range, education_level, tech_background,
                                  focus_area, primary_goal, learning_mode, learning_speed,
                                  time_per_week, preferred_language, created_at, updated_at
                        """,
                        (
                            str(user_id),
                            profile_to_create.age_range.value if profile_to_create.age_range else None,
                            profile_to_create.education_level.value if profile_to_create.education_level else None,
                            profile_to_create.tech_background.value if profile_to_create.tech_background else None,
                            profile_to_create.focus_area.value if profile_to_create.focus_area else None,
                            profile_to_create.primary_goal.value if profile_to_create.primary_goal else None,
                            profile_to_create.learning_mode.value if profile_to_create.learning_mode else None,
                            profile_to_create.learning_speed.value if profile_to_create.learning_speed else None,
                            profile_to_create.time_per_week,
                            profile_to_create.preferred_language,
                        ),
                    )
                    row = cur.fetchone()

                    if row:
                        return self._row_to_profile(row)
                    return None

        except ValidationError as e:
            logger.error(f"Profile data validation error: {e}")
            return None
        except Exception as e:
            logger.error(f"Error creating profile for user {user_id}: {e}")
            return None

    async def get_profile(self, user_id: str) -> Optional[UserProfile]:
        """
        Get user profile by user ID.

        Args:
            user_id: User ID from Better Auth

        Returns:
            UserProfile or None if not found
        """
        try:
            with get_db_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute(
                        """
                        SELECT id, user_id, age_range, education_level, tech_background,
                               focus_area, primary_goal, learning_mode, learning_speed,
                               time_per_week, preferred_language, created_at, updated_at
                        FROM user_profile
                        WHERE user_id = %s
                        LIMIT 1
                        """,
                        (str(user_id),),
                    )
                    row = cur.fetchone()

                    if row:
                        return self._row_to_profile(row)
                    return None

        except Exception as e:
            logger.error(f"Error retrieving profile for user {user_id}: {e}")
            return None

    async def update_profile(self, user_id: str, profile_data: Dict[str, Any]) -> Optional[UserProfile]:
        """
        Update user profile.

        Args:
            user_id: User ID from Better Auth
            profile_data: Fields to update

        Returns:
            Updated UserProfile or None if update fails
        """
        try:
            # Validate input data against Pydantic model
            data_to_validate = profile_data.copy()
            data_to_validate["user_id"] = str(user_id)

            validated_model = UserProfile(**data_to_validate)

            # Build dynamic UPDATE query based on provided fields
            update_fields = []
            values = []

            if validated_model.age_range is not None:
                update_fields.append("age_range = %s")
                values.append(validated_model.age_range.value)
            if validated_model.education_level is not None:
                update_fields.append("education_level = %s")
                values.append(validated_model.education_level.value)
            if validated_model.tech_background is not None:
                update_fields.append("tech_background = %s")
                values.append(validated_model.tech_background.value)
            if validated_model.focus_area is not None:
                update_fields.append("focus_area = %s")
                values.append(validated_model.focus_area.value)
            if validated_model.primary_goal is not None:
                update_fields.append("primary_goal = %s")
                values.append(validated_model.primary_goal.value)
            if validated_model.learning_mode is not None:
                update_fields.append("learning_mode = %s")
                values.append(validated_model.learning_mode.value)
            if validated_model.learning_speed is not None:
                update_fields.append("learning_speed = %s")
                values.append(validated_model.learning_speed.value)
            if validated_model.time_per_week is not None:
                update_fields.append("time_per_week = %s")
                values.append(validated_model.time_per_week)
            if "preferred_language" in profile_data:
                update_fields.append("preferred_language = %s")
                values.append(validated_model.preferred_language)

            if not update_fields:
                # Nothing to update, return existing profile
                return await self.get_profile(user_id)

            # Add updated_at
            update_fields.append("updated_at = NOW()")

            # Add user_id for WHERE clause
            values.append(str(user_id))

            with get_db_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute(
                        f"""
                        UPDATE user_profile
                        SET {', '.join(update_fields)}
                        WHERE user_id = %s
                        RETURNING id, user_id, age_range, education_level, tech_background,
                                  focus_area, primary_goal, learning_mode, learning_speed,
                                  time_per_week, preferred_language, created_at, updated_at
                        """,
                        tuple(values),
                    )
                    row = cur.fetchone()

                    if row:
                        return self._row_to_profile(row)
                    return None

        except ValidationError as e:
            logger.error(f"Profile data validation error during update: {e}")
            return None
        except Exception as e:
            logger.error(f"Error updating profile for user {user_id}: {e}")
            return None

    async def profile_exists(self, user_id: str) -> bool:
        """
        Check if profile exists for user.

        Args:
            user_id: User ID from Better Auth

        Returns:
            True if profile exists, False otherwise
        """
        try:
            with get_db_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute(
                        "SELECT 1 FROM user_profile WHERE user_id = %s LIMIT 1",
                        (str(user_id),),
                    )
                    return cur.fetchone() is not None
        except Exception as e:
            logger.error(f"Error checking profile existence for user {user_id}: {e}")
            return False

    def _row_to_profile(self, row: tuple) -> UserProfile:
        """Convert database row to UserProfile model."""
        # Column order: id, user_id, age_range, education_level, tech_background,
        #               focus_area, primary_goal, learning_mode, learning_speed,
        #               time_per_week, preferred_language, created_at, updated_at
        return UserProfile(
            user_id=row[1],
            age_range=row[2],
            education_level=row[3],
            tech_background=row[4],
            focus_area=row[5],
            primary_goal=row[6],
            learning_mode=row[7],
            learning_speed=row[8],
            time_per_week=row[9],
            preferred_language=row[10] or "en",
        )
