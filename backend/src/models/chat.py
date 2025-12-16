"""
Chat persistence models for Neon database integration.

These models map to the chat_threads and chat_thread_items tables
and are compatible with the ChatKit framework's ThreadMetadata and ThreadItem types.
"""

from datetime import datetime
from typing import Any, Dict, Literal, Optional
from pydantic import BaseModel, Field, field_validator


class ChatThreadMetadata(BaseModel):
    """
    Thread metadata matching ChatKit ThreadMetadata structure.

    Represents a conversation session containing multiple messages.
    """
    id: str = Field(..., pattern=r"^thread_[a-z0-9]+$", description="Unique thread identifier")
    user_id: Optional[str] = Field(None, description="Owner user ID (NULL for anonymous sessions)")
    title: Optional[str] = Field(None, max_length=255, description="Optional thread title")
    metadata: Dict[str, Any] = Field(default_factory=dict, description="Arbitrary metadata for Agent SDK")
    created_at: datetime = Field(..., description="Thread creation timestamp")
    updated_at: datetime = Field(..., description="Last update timestamp")

    class Config:
        json_schema_extra = {
            "example": {
                "id": "thread_abc123def456",
                "user_id": "user_xyz789",
                "title": "Learning about ROS2",
                "metadata": {"previous_response_id": "resp_123"},
                "created_at": "2025-12-16T10:30:00Z",
                "updated_at": "2025-12-16T10:35:00Z"
            }
        }


class ChatThreadItem(BaseModel):
    """
    Thread item matching ChatKit ThreadItem structure.

    Represents a single message or item within a thread.
    """
    id: str = Field(..., description="Unique item identifier")
    thread_id: str = Field(..., pattern=r"^thread_[a-z0-9]+$", description="Parent thread ID")
    type: Literal["message", "tool_call", "task", "workflow", "attachment"] = Field(
        ..., description="Item type"
    )
    role: Optional[Literal["user", "assistant", "system"]] = Field(
        None, description="Message role (NULL for non-message types)"
    )
    content: Dict[str, Any] = Field(..., description="Full item content as JSON")
    created_at: datetime = Field(..., description="Item creation timestamp")
    n_tokens: Optional[int] = Field(None, description="Token count for analytics")

    @field_validator("content")
    @classmethod
    def validate_content_size(cls, v: Dict[str, Any]) -> Dict[str, Any]:
        """Validate content does not exceed 32KB when serialized."""
        import json
        content_str = json.dumps(v)
        size_bytes = len(content_str.encode('utf-8'))
        if size_bytes > 32768:  # 32KB
            raise ValueError(f"Content size {size_bytes} bytes exceeds 32KB limit")
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "id": "message_abc123",
                "thread_id": "thread_xyz789",
                "type": "message",
                "role": "user",
                "content": {"text": "What is ROS2?"},
                "created_at": "2025-12-16T10:30:00Z",
                "n_tokens": 5
            }
        }


class ChatPage(BaseModel):
    """
    Paginated response for threads or items.

    Matches ChatKit Page structure for cursor-based pagination.
    """
    data: list = Field(..., description="Page data (threads or items)")
    has_more: bool = Field(..., description="Whether more pages exist")
    after: Optional[str] = Field(None, description="Cursor for next page")

    class Config:
        json_schema_extra = {
            "example": {
                "data": [],
                "has_more": False,
                "after": None
            }
        }
