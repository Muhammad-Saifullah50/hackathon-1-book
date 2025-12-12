from typing import Optional
from pydantic import BaseModel, EmailStr
import uuid

class User(BaseModel):
    id: str
    email: EmailStr
    is_active: bool = True

class Credentials(BaseModel):
    email: EmailStr
    password: str

class AuthResponse(BaseModel):
    user: User
    access_token: Optional[str] = None
    refresh_token: Optional[str] = None
