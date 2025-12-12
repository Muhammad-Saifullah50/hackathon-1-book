import pytest
from httpx import AsyncClient
from main import app # Assuming main.py is in the root of backend for import
from uuid import uuid4
from backend.src.api.profile.routes import get_current_user

# Mock for `get_current_user` dependency in profile routes
def override_get_current_user():
    return User(id=str(uuid4()), email="test_user@example.com")

# Mock the dependency for testing
app.dependency_overrides[get_current_user] = override_get_current_user


@pytest.mark.asyncio
async def test_create_profile_success():
    current_user_id = app.dependency_overrides[get_current_user]().id
    profile_data = {
        "user_id": current_user_id,
        "age_range": "25_34",
        "education_level": "masters",
        "tech_background": "software_engineer",
        "primary_goal": "career_switch",
        "learning_mode": "visual",
        "learning_speed": "intensive",
        "time_per_week": 20,
        "preferred_language": "en"
    }
    async with AsyncClient(app=app, base_url="http://test") as ac:
        response = await ac.post("/profile", json=profile_data)
    assert response.status_code == 201
    assert response.json()["user_id"] == current_user_id
    assert response.json()["age_range"] == "25_34"

@pytest.mark.asyncio
async def test_create_profile_for_another_user_fails():
    current_user_id = app.dependency_overrides[get_current_user]().id
    another_user_id = str(uuid4())
    profile_data = {
        "user_id": another_user_id, # Trying to create for a different user
        "age_range": "25_34",
        "education_level": "masters",
        "tech_background": "software_engineer",
        "primary_goal": "career_switch",
        "learning_mode": "visual",
        "learning_speed": "intensive",
        "time_per_week": 20,
        "preferred_language": "en"
    }
    async with AsyncClient(app=app, base_url="http://test") as ac:
        response = await ac.post("/profile", json=profile_data)
    assert response.status_code == 403
    assert "Cannot create profile for another user." in response.json()["detail"]

@pytest.mark.asyncio
async def test_get_profile_success():
    current_user_id = app.dependency_overrides[get_current_user]().id
    # First create a profile
    profile_data = {
        "user_id": current_user_id,
        "age_range": "18_24",
        "education_level": "undergrad",
        "tech_background": "student",
        "primary_goal": "academic",
        "learning_mode": "textual",
        "learning_speed": "balanced",
        "time_per_week": 10,
        "preferred_language": "ur"
    }
    async with AsyncClient(app=app, base_url="http://test") as ac:
        await ac.post("/profile", json=profile_data)
        # Then get it
        response = await ac.get("/profile")
    assert response.status_code == 200
    assert response.json()["user_id"] == current_user_id
    assert response.json()["preferred_language"] == "ur"


@pytest.mark.asyncio
async def test_get_profile_not_found():
    # This test assumes the mock supabase client is clean for this particular user.
    # In a real test suite, you'd need to ensure test isolation.
    # For now, if the profile isn't created in this test, it should return 404.
    async with AsyncClient(app=app, base_url="http://test") as ac:
        response = await ac.get("/profile")
    assert response.status_code == 404
    assert "Profile not found" in response.json()["detail"]

@pytest.mark.asyncio
async def test_update_profile_success():
    current_user_id = app.dependency_overrides[get_current_user]().id
    # First create a profile
    initial_profile_data = {
        "user_id": current_user_id,
        "age_range": "under_18",
        "education_level": "high_school",
        "tech_background": "hobbyist",
        "primary_goal": "hobby_project",
        "learning_mode": "code_first",
        "learning_speed": "casual",
        "time_per_week": 5,
        "preferred_language": "en"
    }
    async with AsyncClient(app=app, base_url="http://test") as ac:
        await ac.post("/profile", json=initial_profile_data)
        
        # Then update it
        update_data = {
            "user_id": current_user_id, # Must include user_id in payload
            "time_per_week": 25,
            "learning_speed": "intensive"
        }
        response = await ac.put("/profile", json=update_data)
    
    assert response.status_code == 200
    assert response.json()["user_id"] == current_user_id
    assert response.json()["time_per_week"] == 25
    assert response.json()["learning_speed"] == "intensive"
    assert response.json()["age_range"] == "under_18" # Should remain from initial data

@pytest.mark.asyncio
async def test_update_profile_for_another_user_fails():
    current_user_id = app.dependency_overrides[get_current_user]().id
    # Create profile for current user
    initial_profile_data = {
        "user_id": current_user_id,
        "age_range": "under_18",
        "education_level": "high_school",
        "tech_background": "hobbyist",
        "primary_goal": "hobby_project",
        "learning_mode": "code_first",
        "learning_speed": "casual",
        "time_per_week": 5,
        "preferred_language": "en"
    }
    async with AsyncClient(app=app, base_url="http://test") as ac:
        await ac.post("/profile", json=initial_profile_data)

        # Try to update for a different user
        another_user_id = str(uuid4())
        update_data = {
            "user_id": another_user_id,
            "time_per_week": 30
        }
        response = await ac.put("/profile", json=update_data)
    assert response.status_code == 403
    assert "Cannot update profile for another user." in response.json()["detail"]
