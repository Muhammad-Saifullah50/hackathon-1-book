from typing import Optional, Tuple
from src.models.auth import User
from supabase import Client

class AuthService:
    def __init__(self, supabase_client: Client):
        self.supabase = supabase_client

    async def register_user(self, email: str, password: str) -> Tuple[Optional[User], Optional[object], Optional[str]]:
        """
        Register a new user.
        Returns: (user, session, error_message)
        - On success: (User, session_or_None, None)
        - On existing user: (None, None, "user_already_exists")
        - On other failure: (None, None, "registration_failed")
        """
        try:
            # Supabase sign_up returns generic response, we need to map it
            response = self.supabase.auth.sign_up({"email": email, "password": password})
            if response and response.user:
                # Check if user already exists - Supabase returns user but with empty identities
                # when signing up with an existing email
                if hasattr(response.user, 'identities') and len(response.user.identities) == 0:
                    return None, None, "user_already_exists"

                # Map Supabase user to our User model
                user = User(id=str(response.user.id), email=response.user.email)
                return user, response.session, None
            return None, None, "registration_failed"
        except Exception as e:
            print(f"Registration failed: {e}")
            return None, None, "registration_failed"

    async def login_user(self, email: str, password: str) -> Tuple[Optional[User], Optional[object]]:
        try:
            response = self.supabase.auth.sign_in_with_password({"email": email, "password": password})
            if response and response.user:
                user = User(id=str(response.user.id), email=response.user.email)
                return user, response.session
            return None, None
        except Exception as e:
            print(f"Login failed: {e}")
            return None, None

    async def logout_user(self, token: str) -> bool:
        try:
            self.supabase.auth.sign_out() # Sign out current session (client tracks it?) 
            # Note: supabase-py client might need token explicitly if stateless
            # But usually it tracks session if initialized with one.
            # For stateless API, we might just rely on token expiration or explicit invalidation if supported.
            # Python client sign_out() signs out the *local* session.
            return True
        except Exception as e:
            print(f"Logout failed: {e}")
            return False

    async def get_current_user(self, token: str) -> Optional[User]:
        try:
            response = self.supabase.auth.get_user(token)
            if response and response.user:
                return User(id=str(response.user.id), email=response.user.email)
            return None
        except Exception as e:
            print(f"Get current user failed: {e}")
            return None
