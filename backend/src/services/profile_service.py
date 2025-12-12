# backend/src/services/profile_service.py

from typing import Optional, Dict, Any
from uuid import UUID
from pydantic import ValidationError
from src.models.profile import UserProfile
from src.models.auth import User # Assuming User model might be needed for context
import os
from supabase import create_client, Client

class ProfileService:
    def __init__(self, supabase_client: Optional[Client] = None):
        if supabase_client:
            self.supabase: Client = supabase_client
        else:
            # Initialize Supabase client if not provided (for actual runtime)
            supabase_url = os.environ.get("SUPABASE_URL")
            supabase_key = os.environ.get("SUPABASE_KEY")
            if not supabase_url or not supabase_key:
                raise ValueError("Supabase URL and Key must be set in environment variables")
            self.supabase: Client = create_client(supabase_url, supabase_key)

    async def create_profile(self, user_id: UUID, profile_data: Dict[str, Any]) -> Optional[UserProfile]:
        try:
            # Ensure user_id is included in the profile data for insertion
            profile_data["user_id"] = str(user_id)
            
            # Validate input data against Pydantic model
            profile_to_create = UserProfile(**profile_data)

            # Convert model to dict with JSON-serializable values (UUID -> str)
            profile_dict = profile_to_create.model_dump(mode='json')
            response = self.supabase.table("profiles").insert(profile_dict).execute()
            
            if response.data:
                return UserProfile(**response.data[0])
            return None
        except ValidationError as e:
            print(f"Profile data validation error: {e}")
            return None
        except Exception as e:
            print(f"Error creating profile for user {user_id}: {e}")
            return None

    async def get_profile(self, user_id: UUID) -> Optional[UserProfile]:
        try:
            # Use limit(1) instead of single() to avoid exception when no rows are found
            response = self.supabase.table("profiles").select("*").eq("user_id", str(user_id)).limit(1).execute()
            if response.data and len(response.data) > 0:
                return UserProfile(**response.data[0])
            return None
        except Exception as e:
            print(f"Error retrieving profile for user {user_id}: {e}")
            return None

    async def update_profile(self, user_id: UUID, profile_data: Dict[str, Any]) -> Optional[UserProfile]:
        try:
            # Validate input data against Pydantic model
            # We need to inject user_id to satisfy the required field constraint for validation
            data_to_validate = profile_data.copy()
            data_to_validate["user_id"] = str(user_id)
            
            validated_model = UserProfile(**data_to_validate)
            
            # Get fields to update, excluding those not set in the input
            update_fields = validated_model.model_dump(exclude_unset=True)
            
            # Remove user_id from update payload as it is the primary key and used in the query
            if "user_id" in update_fields:
                del update_fields["user_id"]

            if not update_fields:
                # Nothing to update
                return await self.get_profile(user_id)

            response = self.supabase.table("profiles").update(update_fields).eq("user_id", str(user_id)).execute()
            
            if response.data:
                return UserProfile(**response.data[0])
            return None
        except ValidationError as e:
            print(f"Profile data validation error during update: {e}")
            return None
        except Exception as e:
            print(f"Error updating profile for user {user_id}: {e}")
            return None
