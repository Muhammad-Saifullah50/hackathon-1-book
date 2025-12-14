#!/usr/bin/env python3
"""
Migration script: Supabase to Neon (Better Auth).

This script migrates:
1. User profiles from Supabase profiles table to Neon user_profile table

Note: User authentication data is NOT migrated - users must re-register
with Better Auth. This script only migrates application-specific profile data.

Usage:
    # Dry run (preview only, no changes)
    python migrate_supabase_to_neon.py --dry-run

    # Full migration
    python migrate_supabase_to_neon.py

    # With batch size
    python migrate_supabase_to_neon.py --batch-size 100

Environment variables required:
    FROM_DATABASE_URL: Supabase PostgreSQL connection string
    TO_DATABASE_URL: Neon PostgreSQL connection string
"""

import os
import sys
import argparse
import logging
from typing import List, Dict, Any, Optional, Tuple
from dataclasses import dataclass
from datetime import datetime

import psycopg2
from psycopg2.extras import RealDictCursor
from dotenv import load_dotenv

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


@dataclass
class MigrationStats:
    """Statistics for migration run."""

    total_profiles: int = 0
    migrated_profiles: int = 0
    skipped_profiles: int = 0
    failed_profiles: int = 0
    errors: List[str] = None

    def __post_init__(self):
        if self.errors is None:
            self.errors = []


class MigrationError(Exception):
    """Custom exception for migration errors."""

    pass


def get_source_connection():
    """Get connection to source (Supabase) database."""
    url = os.environ.get("FROM_DATABASE_URL")
    if not url:
        raise MigrationError("FROM_DATABASE_URL environment variable not set")
    return psycopg2.connect(url, cursor_factory=RealDictCursor)


def get_target_connection():
    """Get connection to target (Neon) database."""
    url = os.environ.get("TO_DATABASE_URL")
    if not url:
        raise MigrationError("TO_DATABASE_URL environment variable not set")
    return psycopg2.connect(url, cursor_factory=RealDictCursor)


def fetch_profiles_from_supabase(
    conn, batch_size: int, offset: int
) -> List[Dict[str, Any]]:
    """
    Fetch profiles from Supabase in batches.

    Args:
        conn: Supabase database connection
        batch_size: Number of records per batch
        offset: Starting offset

    Returns:
        List of profile dictionaries
    """
    with conn.cursor() as cur:
        cur.execute(
            """
            SELECT
                user_id,
                age_range,
                education_level,
                tech_background,
                primary_goal,
                learning_mode,
                learning_speed,
                time_per_week,
                preferred_language,
                created_at,
                updated_at
            FROM profiles
            ORDER BY created_at
            LIMIT %s OFFSET %s
            """,
            (batch_size, offset),
        )
        return cur.fetchall()


def get_total_profiles_count(conn) -> int:
    """Get total count of profiles in Supabase."""
    with conn.cursor() as cur:
        cur.execute("SELECT COUNT(*) as count FROM profiles")
        result = cur.fetchone()
        return result["count"] if result else 0


def check_user_exists_in_neon(conn, user_id: str) -> bool:
    """
    Check if user exists in Better Auth user table.

    Note: Users must register with Better Auth before their profile can be migrated.
    This check ensures we don't create orphan profiles.
    """
    with conn.cursor() as cur:
        cur.execute('SELECT 1 FROM "user" WHERE id = %s', (user_id,))
        return cur.fetchone() is not None


def check_profile_exists_in_neon(conn, user_id: str) -> bool:
    """Check if profile already exists in Neon."""
    with conn.cursor() as cur:
        cur.execute("SELECT 1 FROM user_profile WHERE user_id = %s", (user_id,))
        return cur.fetchone() is not None


def insert_profile_to_neon(conn, profile: Dict[str, Any]) -> bool:
    """
    Insert profile into Neon database.

    Args:
        conn: Neon database connection
        profile: Profile data dictionary

    Returns:
        True if successful, False otherwise
    """
    with conn.cursor() as cur:
        cur.execute(
            """
            INSERT INTO user_profile (
                user_id,
                age_range,
                education_level,
                tech_background,
                primary_goal,
                learning_mode,
                learning_speed,
                time_per_week,
                preferred_language,
                created_at,
                updated_at
            )
            VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
            """,
            (
                str(profile["user_id"]),
                profile.get("age_range"),
                profile.get("education_level"),
                profile.get("tech_background"),
                profile.get("primary_goal"),
                profile.get("learning_mode"),
                profile.get("learning_speed"),
                profile.get("time_per_week"),
                profile.get("preferred_language", "en"),
                profile.get("created_at", datetime.now()),
                profile.get("updated_at", datetime.now()),
            ),
        )
        return True


def migrate_profiles(
    batch_size: int = 50, dry_run: bool = False
) -> MigrationStats:
    """
    Migrate profiles from Supabase to Neon.

    Args:
        batch_size: Number of records to process per batch
        dry_run: If True, don't actually insert records

    Returns:
        MigrationStats with results
    """
    stats = MigrationStats()

    logger.info("Starting profile migration...")
    logger.info(f"Batch size: {batch_size}")
    logger.info(f"Dry run: {dry_run}")

    source_conn = None
    target_conn = None

    try:
        source_conn = get_source_connection()
        target_conn = get_target_connection()

        # Get total count
        stats.total_profiles = get_total_profiles_count(source_conn)
        logger.info(f"Total profiles to migrate: {stats.total_profiles}")

        if stats.total_profiles == 0:
            logger.info("No profiles to migrate")
            return stats

        offset = 0
        while True:
            profiles = fetch_profiles_from_supabase(source_conn, batch_size, offset)

            if not profiles:
                break

            logger.info(f"Processing batch: offset={offset}, count={len(profiles)}")

            for profile in profiles:
                user_id = str(profile["user_id"])

                try:
                    # Check if profile already exists in target
                    if check_profile_exists_in_neon(target_conn, user_id):
                        logger.debug(f"Profile already exists for user {user_id}, skipping")
                        stats.skipped_profiles += 1
                        continue

                    # Note: We skip the user existence check for migration
                    # In a real scenario, you'd want users to re-register first
                    # For now, we migrate profiles regardless of user table state

                    if dry_run:
                        logger.info(f"[DRY RUN] Would migrate profile for user {user_id}")
                        stats.migrated_profiles += 1
                    else:
                        insert_profile_to_neon(target_conn, profile)
                        stats.migrated_profiles += 1
                        logger.debug(f"Migrated profile for user {user_id}")

                except Exception as e:
                    error_msg = f"Failed to migrate profile for user {user_id}: {e}"
                    logger.error(error_msg)
                    stats.errors.append(error_msg)
                    stats.failed_profiles += 1

            # Commit after each batch (if not dry run)
            if not dry_run:
                target_conn.commit()

            offset += batch_size

        logger.info("Migration completed!")
        logger.info(f"Total: {stats.total_profiles}")
        logger.info(f"Migrated: {stats.migrated_profiles}")
        logger.info(f"Skipped: {stats.skipped_profiles}")
        logger.info(f"Failed: {stats.failed_profiles}")

        return stats

    except Exception as e:
        logger.error(f"Migration failed: {e}")
        if target_conn:
            target_conn.rollback()
        raise

    finally:
        if source_conn:
            source_conn.close()
        if target_conn:
            target_conn.close()


def verify_migration() -> Tuple[int, int]:
    """
    Verify migration by comparing record counts.

    Returns:
        Tuple of (source_count, target_count)
    """
    source_conn = get_source_connection()
    target_conn = get_target_connection()

    try:
        source_count = get_total_profiles_count(source_conn)

        with target_conn.cursor() as cur:
            cur.execute("SELECT COUNT(*) as count FROM user_profile")
            result = cur.fetchone()
            target_count = result["count"] if result else 0

        return source_count, target_count

    finally:
        source_conn.close()
        target_conn.close()


def rollback_migration():
    """
    Rollback migration by deleting all migrated profiles.

    Warning: This is destructive! Use with caution.
    """
    logger.warning("Rolling back migration - this will delete all profiles in Neon!")

    target_conn = get_target_connection()
    try:
        with target_conn.cursor() as cur:
            cur.execute("DELETE FROM user_profile")
            deleted = cur.rowcount
            target_conn.commit()
            logger.info(f"Deleted {deleted} profiles from Neon")
            return deleted
    finally:
        target_conn.close()


def main():
    """Main entry point."""
    load_dotenv()

    parser = argparse.ArgumentParser(
        description="Migrate profiles from Supabase to Neon"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Preview migration without making changes",
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=50,
        help="Number of records per batch (default: 50)",
    )
    parser.add_argument(
        "--verify",
        action="store_true",
        help="Verify migration by comparing counts",
    )
    parser.add_argument(
        "--rollback",
        action="store_true",
        help="Rollback migration (delete all profiles in Neon)",
    )

    args = parser.parse_args()

    try:
        if args.verify:
            source, target = verify_migration()
            logger.info(f"Source (Supabase) profiles: {source}")
            logger.info(f"Target (Neon) profiles: {target}")
            if source == target:
                logger.info("Migration verified: counts match!")
            else:
                logger.warning(f"Counts differ by {abs(source - target)}")
            return

        if args.rollback:
            confirm = input("Are you sure you want to delete all profiles? (yes/no): ")
            if confirm.lower() == "yes":
                rollback_migration()
            else:
                logger.info("Rollback cancelled")
            return

        stats = migrate_profiles(batch_size=args.batch_size, dry_run=args.dry_run)

        if stats.errors:
            logger.warning(f"Migration completed with {len(stats.errors)} errors")
            for error in stats.errors[:10]:  # Show first 10 errors
                logger.warning(f"  - {error}")

        # Exit with error code if failures occurred
        if stats.failed_profiles > 0:
            sys.exit(1)

    except MigrationError as e:
        logger.error(f"Migration error: {e}")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
