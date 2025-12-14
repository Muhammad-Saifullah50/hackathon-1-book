# backend/src/models/profile.py

from typing import Optional
from pydantic import BaseModel, Field
from enum import Enum

# Define string enums matching database CHECK constraints
class AgeRange(str, Enum):
    UNDER_18 = "under_18"
    _18_24 = "18_24"
    _25_34 = "25_34"
    _35_44 = "35_44"
    _45_PLUS = "45_plus"

class EducationLevel(str, Enum):
    HIGH_SCHOOL = "high_school"
    BACHELORS = "bachelors"
    MASTERS = "masters"
    PHD = "phd"
    SELF_TAUGHT = "self_taught"

class TechBackground(str, Enum):
    NONE = "none"
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"

class FocusArea(str, Enum):
    HARDWARE = "hardware"
    SOFTWARE = "software"

class PrimaryGoal(str, Enum):
    CAREER = "career"
    RESEARCH = "research"
    HOBBY = "hobby"
    EDUCATION = "education"

class LearningMode(str, Enum):
    VISUAL = "visual"
    READING = "reading"
    HANDS_ON = "hands_on"
    MIXED = "mixed"

class LearningSpeed(str, Enum):
    THOROUGH = "thorough"
    BALANCED = "balanced"
    ACCELERATED = "accelerated"

class UserProfile(BaseModel):
    user_id: str  # Better Auth uses NanoID strings, not UUIDs
    age_range: Optional[AgeRange] = None
    education_level: Optional[EducationLevel] = None
    tech_background: Optional[TechBackground] = None
    focus_area: Optional[FocusArea] = None
    primary_goal: Optional[PrimaryGoal] = None
    learning_mode: Optional[LearningMode] = None
    learning_speed: Optional[LearningSpeed] = None
    time_per_week: Optional[int] = Field(None, ge=0)
    preferred_language: str = "en"