"""
Database connection service for Neon PostgreSQL.

Provides connection management with:
- Connection pooling (via psycopg2 connections)
- SSL/TLS encryption (required by Neon)
- Error handling and logging
- Context manager for automatic cleanup
"""

import os
import logging
from contextlib import contextmanager
from typing import Generator, Optional

import psycopg2
from psycopg2 import pool, OperationalError
from psycopg2.extensions import connection as PgConnection

logger = logging.getLogger(__name__)

# Connection pool (initialized lazily)
_connection_pool: Optional[pool.ThreadedConnectionPool] = None


def get_database_url() -> str:
    """Get database URL from environment."""
    url = os.environ.get("DATABASE_URL")
    if not url:
        raise ValueError("DATABASE_URL environment variable is not set")
    return url


def init_connection_pool(min_conn: int = 1, max_conn: int = 10) -> pool.ThreadedConnectionPool:
    """
    Initialize the connection pool.

    Args:
        min_conn: Minimum connections to keep open
        max_conn: Maximum connections allowed

    Returns:
        ThreadedConnectionPool instance
    """
    global _connection_pool

    if _connection_pool is not None:
        return _connection_pool

    database_url = get_database_url()

    try:
        _connection_pool = pool.ThreadedConnectionPool(
            minconn=min_conn,
            maxconn=max_conn,
            dsn=database_url,
            connect_timeout=10,  # 10 second timeout
            # SSL is handled via sslmode in connection string
        )
        logger.info("Database connection pool initialized successfully")
        return _connection_pool
    except OperationalError as e:
        logger.error(f"Failed to initialize database connection pool: {e}")
        raise


def close_connection_pool() -> None:
    """Close all connections in the pool."""
    global _connection_pool

    if _connection_pool is not None:
        _connection_pool.closeall()
        _connection_pool = None
        logger.info("Database connection pool closed")


@contextmanager
def get_db_connection() -> Generator[PgConnection, None, None]:
    """
    Context manager for database connections.

    Automatically returns connection to pool after use.

    Usage:
        with get_db_connection() as conn:
            with conn.cursor() as cur:
                cur.execute("SELECT * FROM users")
                results = cur.fetchall()

    Yields:
        psycopg2 connection object

    Raises:
        OperationalError: If connection cannot be established
    """
    global _connection_pool

    # Initialize pool if needed
    if _connection_pool is None:
        init_connection_pool()

    conn = None
    try:
        conn = _connection_pool.getconn()
        if conn is None:
            raise OperationalError("Failed to get connection from pool")

        yield conn

        # Commit if no exception occurred
        conn.commit()

    except Exception as e:
        if conn is not None:
            conn.rollback()
        logger.error(f"Database operation failed: {e}")
        raise

    finally:
        if conn is not None:
            _connection_pool.putconn(conn)


def check_database_connection() -> bool:
    """
    Check if database connection is working.

    Returns:
        True if connection successful, False otherwise
    """
    try:
        with get_db_connection() as conn:
            with conn.cursor() as cur:
                cur.execute("SELECT 1")
                result = cur.fetchone()
                return result is not None and result[0] == 1
    except Exception as e:
        logger.error(f"Database health check failed: {e}")
        return False


def get_database_version() -> Optional[str]:
    """
    Get PostgreSQL version string.

    Returns:
        Version string or None if query fails
    """
    try:
        with get_db_connection() as conn:
            with conn.cursor() as cur:
                cur.execute("SELECT version()")
                result = cur.fetchone()
                return result[0] if result else None
    except Exception as e:
        logger.error(f"Failed to get database version: {e}")
        return None
