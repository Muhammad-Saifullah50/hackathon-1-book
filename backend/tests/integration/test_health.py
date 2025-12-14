"""
Integration tests for health check endpoint.

Tests:
- Health endpoint returns 200 OK
- Health endpoint includes database status
- Health endpoint reports connection errors
"""

import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch

# These tests will FAIL until implementation is complete (TDD approach)


class TestHealthEndpoint:
    """Integration tests for /health endpoint."""

    @pytest.fixture
    def client(self):
        """Create test client for FastAPI app."""
        from main import app
        return TestClient(app)

    def test_health_endpoint_returns_200(self, client):
        """Test that health endpoint returns 200 OK."""
        with patch("src.services.database.check_database_connection", return_value=True):
            response = client.get("/health")
            assert response.status_code == 200

    def test_health_endpoint_includes_status_ok(self, client):
        """Test that health endpoint includes status: ok when healthy."""
        with patch("src.services.database.check_database_connection", return_value=True):
            response = client.get("/health")
            data = response.json()
            assert data["status"] == "ok"

    def test_health_endpoint_includes_database_status(self, client):
        """Test that health endpoint includes database connection status."""
        with patch("src.services.database.check_database_connection", return_value=True):
            response = client.get("/health")
            data = response.json()
            assert "database" in data
            assert data["database"] == "connected"

    def test_health_endpoint_reports_database_disconnected(self, client):
        """Test that health endpoint reports when database is disconnected."""
        with patch("src.services.database.check_database_connection", return_value=False):
            response = client.get("/health")
            data = response.json()
            # Should still return 200 but indicate database is disconnected
            assert response.status_code == 200
            assert data["database"] == "disconnected"

    def test_health_endpoint_includes_service_name(self, client):
        """Test that health endpoint includes service name."""
        with patch("src.services.database.check_database_connection", return_value=True):
            response = client.get("/health")
            data = response.json()
            assert "service" in data
