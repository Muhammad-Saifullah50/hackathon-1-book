from fastapi import APIRouter, Depends, HTTPException, status, Header
from fastapi.responses import Response
from typing import Annotated, Optional
import os
from supabase import create_client, Client

from src.models.auth import User, Credentials, AuthResponse
from src.services.auth_service import AuthService

router = APIRouter(prefix="/auth", tags=["auth"])

def get_supabase_client() -> Client:
    supabase_url = os.environ.get("SUPABASE_URL")
    supabase_key = os.environ.get("SUPABASE_KEY")
    if not supabase_url or not supabase_key:
        raise ValueError("Supabase URL and Key must be set")
    return create_client(supabase_url, supabase_key)

# Dependency to get AuthService instance
def get_auth_service(supabase: Annotated[Client, Depends(get_supabase_client)]):
    return AuthService(supabase_client=supabase)

@router.post("/signup", response_model=AuthResponse, status_code=status.HTTP_201_CREATED)
async def signup(credentials: Credentials, auth_service: Annotated[AuthService, Depends(get_auth_service)]):
    user, session, error = await auth_service.register_user(credentials.email, credentials.password)

    if error == "user_already_exists":
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="User already exists. Please login instead."
        )

    if user is None:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Registration failed."
        )

    access_token = session.access_token if session else None
    refresh_token = session.refresh_token if session else None

    return AuthResponse(user=user, access_token=access_token, refresh_token=refresh_token)

@router.post("/login", response_model=AuthResponse)
async def login(credentials: Credentials, auth_service: Annotated[AuthService, Depends(get_auth_service)]):
    user, session = await auth_service.login_user(credentials.email, credentials.password)
    if user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password."
        )
    
    access_token = session.access_token if session else None
    refresh_token = session.refresh_token if session else None
    
    return AuthResponse(user=user, access_token=access_token, refresh_token=refresh_token)

@router.post("/logout", status_code=status.HTTP_200_OK)
async def logout(
    response: Response, 
    auth_service: Annotated[AuthService, Depends(get_auth_service)],
    authorization: Annotated[Optional[str], Header()] = None
):
    if authorization:
        token = authorization.replace("Bearer ", "")
        await auth_service.logout_user(token)
    return {"message": "Logout successful"}

@router.get("/me", response_model=User)
async def get_current_user_endpoint(
    auth_service: Annotated[AuthService, Depends(get_auth_service)],
    authorization: Annotated[Optional[str], Header()] = None
):
    if not authorization:
        raise HTTPException(status_code=401, detail="Missing Authorization header")
    
    token = authorization.replace("Bearer ", "")
    user = await auth_service.get_current_user(token)
    
    if user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated or invalid token."
        )
    return user