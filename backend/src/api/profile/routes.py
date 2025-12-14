"""
Profile API routes.

Uses JWT authentication from Better Auth.
Profile data stored in Neon PostgreSQL.
"""

from fastapi import APIRouter, Depends, HTTPException, status
from typing import Annotated

from src.models.profile import UserProfile
from src.services.profile_service import ProfileService
from src.api.auth.dependencies import get_current_user, CurrentUser

router = APIRouter(prefix="/profile", tags=["profile"])


# Dependency to get ProfileService instance
def get_profile_service() -> ProfileService:
    """Get ProfileService instance (uses Neon database)."""
    return ProfileService()


@router.post("", response_model=UserProfile, status_code=status.HTTP_201_CREATED)
async def create_profile(
    profile_data: UserProfile,
    current_user: Annotated[CurrentUser, Depends(get_current_user)],
    profile_service: Annotated[ProfileService, Depends(get_profile_service)],
):
    """
    Create a new profile for the authenticated user.

    - Requires valid JWT token
    - Profile user_id must match authenticated user
    - Returns 409 if profile already exists
    - Returns 201 with created profile on success
    """
    # Ensure the profile being created is for the current authenticated user
    if profile_data.user_id != current_user.id:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Cannot create profile for another user.",
        )

    # Check if profile already exists
    if await profile_service.profile_exists(current_user.id):
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="Profile already exists for this user.",
        )

    profile = await profile_service.create_profile(
        current_user.id, profile_data.model_dump()
    )
    if profile is None:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Failed to create profile.",
        )
    return profile


@router.get("", response_model=UserProfile)
async def get_profile(
    current_user: Annotated[CurrentUser, Depends(get_current_user)],
    profile_service: Annotated[ProfileService, Depends(get_profile_service)],
):
    """
    Get the profile for the authenticated user.

    - Requires valid JWT token
    - Returns 404 if profile not found
    - Returns profile data on success
    """
    profile = await profile_service.get_profile(current_user.id)
    if profile is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Profile not found for this user.",
        )
    return profile


@router.put("", response_model=UserProfile)
async def update_profile(
    profile_data: UserProfile,
    current_user: Annotated[CurrentUser, Depends(get_current_user)],
    profile_service: Annotated[ProfileService, Depends(get_profile_service)],
):
    """
    Update the profile for the authenticated user.

    - Requires valid JWT token
    - Profile user_id must match authenticated user
    - Returns 404 if profile not found
    - Returns updated profile on success
    """
    # Ensure the profile being updated is for the current authenticated user
    if profile_data.user_id != current_user.id:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Cannot update profile for another user.",
        )

    # Check if profile exists
    if not await profile_service.profile_exists(current_user.id):
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Profile not found for this user.",
        )

    updated_profile = await profile_service.update_profile(
        current_user.id, profile_data.model_dump(exclude_unset=True)
    )
    if updated_profile is None:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to update profile.",
        )
    return updated_profile
