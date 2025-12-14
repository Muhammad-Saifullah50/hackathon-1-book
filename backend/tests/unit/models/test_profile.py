import pytest
from pydantic import ValidationError
from uuid import uuid4
from src.models.profile import UserProfile, AgeRange, EducationLevel, TechBackground, FocusArea, PrimaryGoal, LearningMode, LearningSpeed

def test_user_profile_creation_valid():
    user_id = uuid4()
    profile = UserProfile(
        user_id=user_id,
        age_range=AgeRange._25_34,
        education_level=EducationLevel.MASTERS,
        tech_background=TechBackground.SOFTWARE_ENGINEER,
        primary_goal=PrimaryGoal.CAREER_SWITCH,
        learning_mode=LearningMode.VISUAL,
        learning_speed=LearningSpeed.INTENSIVE,
        time_per_week=10,
        preferred_language="en"
    )
    assert profile.user_id == user_id
    assert profile.age_range == AgeRange._25_34
    assert profile.preferred_language == "en"
    assert profile.time_per_week == 10

def test_user_profile_creation_minimal():
    user_id = uuid4()
    profile = UserProfile(user_id=user_id)
    assert profile.user_id == user_id
    assert profile.age_range is None
    assert profile.preferred_language == "en" # Default value

def test_user_profile_time_per_week_negative_fails():
    user_id = uuid4()
    with pytest.raises(ValidationError):
        UserProfile(user_id=user_id, time_per_week=-5)

def test_user_profile_invalid_enum_fails():
    user_id = uuid4()
    with pytest.raises(ValidationError):
        UserProfile(user_id=user_id, age_range="invalid_age")
    with pytest.raises(ValidationError):
        UserProfile(user_id=user_id, education_level="invalid_edu")

def test_user_profile_preferred_language_default():
    user_id = uuid4()
    profile = UserProfile(user_id=user_id)
    assert profile.preferred_language == "en"

def test_user_profile_preferred_language_custom():
    user_id = uuid4()
    profile = UserProfile(user_id=user_id, preferred_language="ur")
    assert profile.preferred_language == "ur"

# Test UUID validation implicitly handled by Pydantic's UUID type
def test_user_id_invalid_type_fails():
    with pytest.raises(ValidationError):
        UserProfile(user_id="not-a-uuid")

def test_user_profile_focus_area_hardware():
    user_id = uuid4()
    profile = UserProfile(user_id=user_id, focus_area=FocusArea.HARDWARE)
    assert profile.focus_area == FocusArea.HARDWARE
    assert profile.focus_area.value == "hardware"

def test_user_profile_focus_area_software():
    user_id = uuid4()
    profile = UserProfile(user_id=user_id, focus_area=FocusArea.SOFTWARE)
    assert profile.focus_area == FocusArea.SOFTWARE
    assert profile.focus_area.value == "software"

def test_user_profile_focus_area_invalid_fails():
    user_id = uuid4()
    with pytest.raises(ValidationError):
        UserProfile(user_id=user_id, focus_area="both")  # Invalid - only hardware or software
