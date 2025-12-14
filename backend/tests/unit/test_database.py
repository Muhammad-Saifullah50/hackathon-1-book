"""
Unit tests for database connection service.

Tests:
- Connection pool initialization
- get_db_connection context manager
- Connection error handling
- Health check functionality
"""

import os
import pytest
from unittest.mock import patch, MagicMock

# These tests will FAIL until implementation is complete (TDD approach)


class TestDatabaseConnection:
    """Tests for database connection service."""

    def test_get_database_url_returns_url_from_env(self):
        """Test that DATABASE_URL is read from environment."""
        from src.services.database import get_database_url

        with patch.dict(os.environ, {"DATABASE_URL": "postgresql://test:test@localhost/testdb"}):
            url = get_database_url()
            assert url == "postgresql://test:test@localhost/testdb"

    def test_get_database_url_raises_without_env(self):
        """Test that missing DATABASE_URL raises ValueError."""
        from src.services.database import get_database_url

        with patch.dict(os.environ, {}, clear=True):
            # Remove DATABASE_URL if it exists
            os.environ.pop("DATABASE_URL", None)
            with pytest.raises(ValueError, match="DATABASE_URL"):
                get_database_url()

    def test_init_connection_pool_creates_pool(self):
        """Test that connection pool is initialized correctly."""
        from src.services.database import init_connection_pool, close_connection_pool

        with patch.dict(os.environ, {"DATABASE_URL": "postgresql://test:test@localhost/testdb"}):
            with patch("src.services.database.pool.ThreadedConnectionPool") as mock_pool_class:
                mock_pool = MagicMock()
                mock_pool_class.return_value = mock_pool

                result = init_connection_pool(min_conn=1, max_conn=5)

                mock_pool_class.assert_called_once()
                assert result is mock_pool

                # Cleanup
                close_connection_pool()

    def test_get_db_connection_yields_connection(self):
        """Test that get_db_connection yields a connection and commits."""
        from src.services.database import get_db_connection, close_connection_pool

        with patch.dict(os.environ, {"DATABASE_URL": "postgresql://test:test@localhost/testdb"}):
            with patch("src.services.database.pool.ThreadedConnectionPool") as mock_pool_class:
                mock_pool = MagicMock()
                mock_conn = MagicMock()
                mock_pool.getconn.return_value = mock_conn
                mock_pool_class.return_value = mock_pool

                with get_db_connection() as conn:
                    assert conn is mock_conn

                mock_conn.commit.assert_called_once()
                mock_pool.putconn.assert_called_once_with(mock_conn)

                # Cleanup
                close_connection_pool()

    def test_get_db_connection_rolls_back_on_exception(self):
        """Test that connection is rolled back when exception occurs."""
        from src.services.database import get_db_connection, close_connection_pool

        with patch.dict(os.environ, {"DATABASE_URL": "postgresql://test:test@localhost/testdb"}):
            with patch("src.services.database.pool.ThreadedConnectionPool") as mock_pool_class:
                mock_pool = MagicMock()
                mock_conn = MagicMock()
                mock_pool.getconn.return_value = mock_conn
                mock_pool_class.return_value = mock_pool

                with pytest.raises(RuntimeError):
                    with get_db_connection() as conn:
                        raise RuntimeError("Test error")

                mock_conn.rollback.assert_called_once()
                mock_pool.putconn.assert_called_once_with(mock_conn)

                # Cleanup
                close_connection_pool()

    def test_check_database_connection_returns_true_when_connected(self):
        """Test health check returns True when database is connected."""
        from src.services.database import check_database_connection, close_connection_pool

        with patch.dict(os.environ, {"DATABASE_URL": "postgresql://test:test@localhost/testdb"}):
            with patch("src.services.database.pool.ThreadedConnectionPool") as mock_pool_class:
                mock_pool = MagicMock()
                mock_conn = MagicMock()
                mock_cursor = MagicMock()
                mock_cursor.fetchone.return_value = (1,)
                mock_conn.cursor.return_value.__enter__ = MagicMock(return_value=mock_cursor)
                mock_conn.cursor.return_value.__exit__ = MagicMock(return_value=False)
                mock_pool.getconn.return_value = mock_conn
                mock_pool_class.return_value = mock_pool

                result = check_database_connection()

                assert result is True

                # Cleanup
                close_connection_pool()

    def test_check_database_connection_returns_false_on_error(self):
        """Test health check returns False when database connection fails."""
        from src.services.database import check_database_connection, close_connection_pool

        with patch.dict(os.environ, {"DATABASE_URL": "postgresql://test:test@localhost/testdb"}):
            with patch("src.services.database.pool.ThreadedConnectionPool") as mock_pool_class:
                mock_pool = MagicMock()
                mock_pool.getconn.side_effect = Exception("Connection failed")
                mock_pool_class.return_value = mock_pool

                result = check_database_connection()

                assert result is False

                # Cleanup
                close_connection_pool()
