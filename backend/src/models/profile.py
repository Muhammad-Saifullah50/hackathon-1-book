# backend/src/models/profile.py

from typing import Optional
from uuid import UUID
from pydantic import BaseModel, Field
from enum import Enum # Import Enum

# Define string enums using Python's Enum class
class AgeRange(str, Enum):
    UNDER_18 = "under_18"
    _18_24 = "18_24"
    _25_34 = "25_34"
    _35_PLUS = "35_plus"

class EducationLevel(str, Enum):
    HIGH_SCHOOL = "high_school"
    UNDERGRAD = "undergrad"
    MASTERS = "masters"
    PHD = "phd"
    SELF_TAUGHT = "self_taught"

class TechBackground(str, Enum):
    SOFTWARE_ENGINEER = "software_engineer"
    HARDWARE_ENGINEER = "hardware_engineer"
    STUDENT = "student"
    HOBBYIST = "hobbyist"

class PrimaryGoal(str, Enum):
    CAREER_SWITCH = "career_switch"
    ACADEMIC = "academic"
    HOBBY_PROJECT = "hobby_project"
    STARTUP_FOUNDER = "startup_founder"

class LearningMode(str, Enum):
    VISUAL = "visual"
    TEXTUAL = "textual"
    CODE_FIRST = "code_first"

class LearningSpeed(str, Enum):
    INTENSIVE = "intensive"
    BALANCED = "balanced"
    CASUAL = "casual"

class UserProfile(BaseModel):
    user_id: UUID
    age_range: Optional[AgeRange] = None
    education_level: Optional[EducationLevel] = None
    tech_background: Optional[TechBackground] = None
    primary_goal: Optional[PrimaryGoal] = None
    learning_mode: Optional[LearningMode] = None
    learning_speed: Optional[LearningSpeed] = None
    time_per_week: Optional[int] = Field(None, ge=0)
    preferred_language: str = "en"

    class Config:
        # Serialize UUID to string for JSON compatibility
        json_encoders = {
            UUID: str
        }