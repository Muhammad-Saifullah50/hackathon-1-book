# backend/src/models/personalization.py
"""
Pydantic models for page personalization feature.
"""

from datetime import date, datetime
from typing import Optional
from pydantic import BaseModel, Field


class PersonalizationHistoryCreate(BaseModel):
    """Create a new personalization history record."""
    user_id: str
    page_url: str = Field(max_length=500)
    profile_hash: str = Field(max_length=64)
    original_content_hash: Optional[str] = Field(None, max_length=64)


class PersonalizationHistoryRecord(BaseModel):
    """Personalization history record from database."""
    id: int
    user_id: str
    page_url: str
    profile_hash: str
    original_content_hash: Optional[str]
    created_at: datetime
    updated_at: datetime

    model_config = {"from_attributes": True}


class PersonalizationQuotaRecord(BaseModel):
    """Daily quota record from database."""
    id: int
    user_id: str
    date: date
    used_count: int
    created_at: datetime
    updated_at: datetime

    model_config = {"from_attributes": True}


class QuotaStatus(BaseModel):
    """Quota status response."""
    limit: int = 5
    used: int
    remaining: int
    resets_at: datetime  # Midnight UTC next day


class PersonalizationRequest(BaseModel):
    """Request to personalize page content."""
    page_url: str = Field(max_length=500)
    page_content: str = Field(min_length=1)
    is_free_repersonalization: bool = False


class PersonalizationResponse(BaseModel):
    """Response with personalized content."""
    personalized_content: str
    profile_hash: str
    original_content_hash: str
    processing_time_ms: int
    quota_remaining: int


class PersonalizationHistoryItem(BaseModel):
    """Single item in personalization history."""
    url: str
    profile_hash: str
    created_at: datetime


class PersonalizationHistoryResponse(BaseModel):
    """User's personalization history."""
    pages: list[PersonalizationHistoryItem]
    current_profile_hash: str


class PageHistoryResponse(BaseModel):
    """Response for single page history check."""
    found: bool
    profile_hash: Optional[str] = None
    original_content_hash: Optional[str] = None
    created_at: Optional[datetime] = None
    is_stale: Optional[bool] = None


class QuotaExceededResponse(BaseModel):
    """Response when quota is exceeded."""
    error: str = "quota_exceeded"
    message: str
    quota_status: QuotaStatus


class ErrorResponse(BaseModel):
    """Generic error response."""
    error: str
    message: str
    details: Optional[dict] = None
    profile_wizard_url: Optional[str] = None
