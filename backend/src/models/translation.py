# backend/src/models/translation.py
"""
Pydantic models for Urdu translation feature.
"""

from datetime import datetime
from typing import Optional
from pydantic import BaseModel, Field


class TranslationRequest(BaseModel):
    """Request to translate page content to Urdu."""
    page_content: str = Field(..., min_length=1, max_length=100000, description="Markdown content to translate")
    page_url: str = Field(..., min_length=1, max_length=500, description="URL of the page being translated")
    source_language: str = Field(default="en", description="Source language code")
    target_language: str = Field(default="ur", description="Target language code (Urdu)")


class TranslationResponse(BaseModel):
    """Response containing translated content."""
    translated_content: str = Field(..., description="Translated markdown content")
    source_language: str = Field(default="en")
    target_language: str = Field(default="ur")
    processing_time_ms: int = Field(..., description="Time taken to translate in milliseconds")
    original_content_hash: str = Field(..., description="SHA-256 hash of original content")
    quota_remaining: int = Field(..., description="Remaining translations for today")
    quota_limit: int = Field(default=5, description="Daily translation limit")


class QuotaStatusResponse(BaseModel):
    """Current quota status for the user/IP."""
    used: int = Field(..., description="Translations used today")
    remaining: int = Field(..., description="Translations remaining today")
    limit: int = Field(default=5, description="Daily limit")
    resets_at: datetime = Field(..., description="When quota resets (midnight UTC)")


class TranslationHistoryResponse(BaseModel):
    """Response for translation history check."""
    has_translation: bool = Field(..., description="Whether page was previously translated")
    original_content_hash: Optional[str] = Field(None, description="Hash of original content at translation time")
    translated_at: Optional[datetime] = Field(None, description="When the page was translated")
    content_changed: bool = Field(default=False, description="Whether original content has changed since translation")


class ErrorResponse(BaseModel):
    """Standard error response."""
    error: str = Field(..., description="Error message")
    code: str = Field(..., description="Error code")
    details: Optional[dict] = Field(None, description="Additional error details")
