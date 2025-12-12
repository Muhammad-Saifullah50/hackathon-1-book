import pytest
from httpx import AsyncClient
from main import app # Assuming main.py is in the root of backend for import

@pytest.mark.asyncio
async def test_signup_success():
    async with AsyncClient(app=app, base_url="http://test") as ac:
        response = await ac.post("/auth/signup", json={"email": "test@example.com", "password": "password123"})
    assert response.status_code == 201
    assert "id" in response.json()
    assert response.json()["email"] == "test@example.com"

@pytest.mark.asyncio
async def test_signup_existing_user_fails():
    async with AsyncClient(app=app, base_url="http://test") as ac:
        # First signup
        await ac.post("/auth/signup", json={"email": "existing@example.com", "password": "password123"})
        # Second signup with same email
        response = await ac.post("/auth/signup", json={"email": "existing@example.com", "password": "password123"})
    assert response.status_code == 400
    assert "Registration failed" in response.json()["detail"]

@pytest.mark.asyncio
async def test_login_success():
    async with AsyncClient(app=app, base_url="http://test") as ac:
        # Register first
        await ac.post("/auth/signup", json={"email": "login@example.com", "password": "password123"})
        # Then login
        response = await ac.post("/auth/login", json={"email": "login@example.com", "password": "password123"})
    assert response.status_code == 200
    assert "id" in response.json()
    assert response.json()["email"] == "login@example.com"

@pytest.mark.asyncio
async def test_login_invalid_credentials():
    async with AsyncClient(app=app, base_url="http://test") as ac:
        response = await ac.post("/auth/login", json={"email": "nonexistent@example.com", "password": "wrongpassword"})
    assert response.status_code == 401
    assert "Incorrect email or password" in response.json()["detail"]

@pytest.mark.asyncio
async def test_logout_success():
    async with AsyncClient(app=app, base_url="http://test") as ac:
        response = await ac.post("/auth/logout")
    assert response.status_code == 200
    assert response.json()["message"] == "Logout successful"

@pytest.mark.asyncio
async def test_get_current_user_authenticated():
    # This test will rely on the MockAuthClient's behavior for 'mock_valid_token'
    async with AsyncClient(app=app, base_url="http://test") as ac:
        response = await ac.get("/auth/me")
    assert response.status_code == 200
    assert "id" in response.json()
    assert response.json()["email"] == "test@example.com" # From MockAuthClient

@pytest.mark.asyncio
async def test_get_current_user_unauthenticated():
    # To simulate unauthenticated, we'd typically not send a token or send an invalid one.
    # Our MockAuthClient returns None if token is not 'mock_valid_token'.
    # For a real test, this would involve manipulating headers or session.
    # For now, we rely on the default behavior of MockAuthClient for unauthenticated state.
    async with AsyncClient(app=app, base_url="http://test") as ac:
        # Temporarily modify MockAuthClient to simulate invalid token
        from src.services.auth_service import get_auth_service
        from src.models.auth import MockAuthClient
        
        original_get_current_user = MockAuthClient.get_current_user
        async def mock_invalid_get_current_user(self, token: str):
            return None
        MockAuthClient.get_current_user = mock_invalid_get_current_user
        
        response = await ac.get("/auth/me")
        
        # Restore original function
        MockAuthClient.get_current_user = original_get_current_user
        
    assert response.status_code == 401
    assert "Not authenticated" in response.json()["detail"]
