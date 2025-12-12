from fastapi import APIRouter, Depends, HTTPException, status, Header
from typing import Annotated, Optional
from uuid import UUID
import os
from supabase import create_client, Client

from src.models.profile import UserProfile
from src.services.profile_service import ProfileService
from src.models.auth import User
# Reuse auth service logic/client if possible, or just init supabase here
# To avoid circular imports or duplication, we can init supabase client here too or import a common provider
# For now, let's use the env vars directly as in ProfileService

router = APIRouter(prefix="/profile", tags=["profile"])

def get_supabase_client() -> Client:
    supabase_url = os.environ.get("SUPABASE_URL")
    supabase_key = os.environ.get("SUPABASE_KEY")
    if not supabase_url or not supabase_key:
        raise ValueError("Supabase URL and Key must be set")
    return create_client(supabase_url, supabase_key)

# Dependency to get ProfileService instance
def get_profile_service(supabase: Annotated[Client, Depends(get_supabase_client)]):
    return ProfileService(supabase_client=supabase)

# Real implementation of get_current_user using Supabase Auth
def get_current_user(
    authorization: Annotated[Optional[str], Header()] = None,
    supabase: Client = Depends(get_supabase_client)
) -> User:
    if not authorization:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Missing Authorization header"
        )
    
    token = authorization.replace("Bearer ", "")
    try:
        response = supabase.auth.get_user(token)
        if not response or not response.user:
             raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid token"
            )
        return User(id=str(response.user.id), email=response.user.email)
    except Exception as e:
        print(f"Auth verification failed: {e}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid authentication credentials"
        )


@router.post("", response_model=UserProfile, status_code=status.HTTP_201_CREATED)
async def create_profile(
    profile_data: UserProfile, # This will be validated by Pydantic
    current_user: Annotated[User, Depends(get_current_user)],
    profile_service: Annotated[ProfileService, Depends(get_profile_service)]
):
    # Ensure the profile being created is for the current authenticated user
    if profile_data.user_id != UUID(current_user.id):
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Cannot create profile for another user."
        )
    
    profile = await profile_service.create_profile(current_user.id, profile_data.model_dump())
    if profile is None:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Failed to create profile."
        )
    return profile

@router.get("", response_model=UserProfile)
async def get_profile(
    current_user: Annotated[User, Depends(get_current_user)],
    profile_service: Annotated[ProfileService, Depends(get_profile_service)]
):
    profile = await profile_service.get_profile(current_user.id)
    if profile is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Profile not found for this user."
        )
    return profile

@router.put("", response_model=UserProfile)
async def update_profile(
    profile_data: UserProfile, # This will be validated by Pydantic (partial update)
    current_user: Annotated[User, Depends(get_current_user)],
    profile_service: Annotated[ProfileService, Depends(get_profile_service)]
):
    # Ensure the profile being updated is for the current authenticated user
    if profile_data.user_id != UUID(current_user.id):
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Cannot update profile for another user."
        )

    updated_profile = await profile_service.update_profile(current_user.id, profile_data.model_dump(exclude_unset=True))
    if updated_profile is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Profile not found or failed to update."
        )
    return updated_profile