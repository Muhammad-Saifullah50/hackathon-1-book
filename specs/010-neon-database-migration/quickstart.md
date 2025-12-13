# Quickstart: Neon Database Migration with Better Auth

**Feature**: 010-neon-database-migration
**Estimated Setup Time**: 30 minutes

## Prerequisites

- [ ] Neon account created at https://neon.tech
- [ ] Neon project and database created
- [ ] Node.js 18+ installed
- [ ] Python 3.12+ installed
- [ ] Git repository cloned

## Step 1: Neon Database Setup

### 1.1 Get Connection String

1. Log into Neon Console
2. Navigate to your project → Connection Details
3. Copy the connection string (format: `postgresql://user:pass@host.neon.tech/dbname?sslmode=require`)

### 1.2 Configure Environment Variables

**Backend** (`backend/.env`):
```dotenv
DATABASE_URL=postgresql://user:pass@ep-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require
```

**Frontend** (`website/.env`):
```dotenv
DATABASE_URL=postgresql://user:pass@ep-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require
AUTH_SECRET=your-32-character-random-string
NEXT_PUBLIC_AUTH_URL=http://localhost:3000
```

## Step 2: Better Auth Setup (Frontend)

### 2.1 Install Dependencies

```bash
cd website
npm install better-auth @better-auth/cli
```

### 2.2 Create Auth Configuration

Create `website/src/lib/auth.ts`:
```typescript
import { betterAuth } from "better-auth";
import { Pool } from "pg";

export const auth = betterAuth({
  database: new Pool({
    connectionString: process.env.DATABASE_URL,
  }),
  emailAndPassword: {
    enabled: true,
  },
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
  },
});
```

### 2.3 Generate Database Schema

```bash
npx @better-auth/cli generate
npx @better-auth/cli migrate
```

### 2.4 Create Auth Client

Create `website/src/lib/auth-client.ts`:
```typescript
import { createAuthClient } from "better-auth/client";
import { jwtClient } from "better-auth/client/plugins";

export const authClient = createAuthClient({
  baseURL: process.env.NEXT_PUBLIC_AUTH_URL || "http://localhost:3000",
  plugins: [jwtClient()],
});
```

### 2.5 Swizzle Root Component

Create `website/src/theme/Root.js`:
```jsx
import React from 'react';
import { AuthProvider } from '../context/AuthContext';

export default function Root({ children }) {
  return (
    <AuthProvider>
      {children}
    </AuthProvider>
  );
}
```

## Step 3: FastAPI JWT Validation Setup (Backend)

### 3.1 Install Dependencies

```bash
cd backend
pip install "PyJWT[crypto]" psycopg2-binary python-dotenv
```

Or add to `requirements.txt`:
```txt
PyJWT[crypto]>=2.8.0
psycopg2-binary>=2.9.9
python-dotenv>=1.0.0
```

### 3.2 Create JWT Service

Create `backend/src/services/jwt_service.py`:
```python
import jwt
from jwt import PyJWKClient
from jwt.exceptions import InvalidTokenError
from functools import lru_cache
import os

JWKS_URL = os.getenv("JWKS_URL", "http://localhost:3000/api/auth/jwks")
ISSUER = os.getenv("JWT_ISSUER", "http://localhost:3000")
AUDIENCE = os.getenv("JWT_AUDIENCE", "http://localhost:3000")

@lru_cache(maxsize=1)
def get_jwks_client():
    return PyJWKClient(JWKS_URL, cache_keys=True)

def validate_jwt(token: str) -> dict:
    """
    Validate JWT token from Better Auth.
    Returns decoded payload with user info.
    Raises InvalidTokenError on failure.
    """
    try:
        jwks_client = get_jwks_client()
        signing_key = jwks_client.get_signing_key_from_jwt(token)

        payload = jwt.decode(
            token,
            signing_key.key,
            algorithms=["RS256"],
            audience=AUDIENCE,
            issuer=ISSUER,
        )
        return payload
    except Exception as e:
        raise InvalidTokenError(f"Token validation failed: {e}")
```

### 3.3 Create Auth Dependency

Create `backend/src/api/auth/dependencies.py`:
```python
from fastapi import HTTPException, Depends, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from src.services.jwt_service import validate_jwt
from jwt.exceptions import InvalidTokenError

security = HTTPBearer()

async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security)
) -> dict:
    """
    FastAPI dependency to validate JWT and return current user.
    """
    token = credentials.credentials
    try:
        payload = validate_jwt(token)
        return {
            "user_id": payload.get("sub"),
            "email": payload.get("email"),
            "name": payload.get("name"),
        }
    except InvalidTokenError as e:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail=str(e),
            headers={"WWW-Authenticate": "Bearer"},
        )
```

### 3.4 Protect Endpoints

Update `backend/src/api/profile/routes.py`:
```python
from fastapi import APIRouter, Depends
from src.api.auth.dependencies import get_current_user

router = APIRouter(prefix="/api/profile", tags=["profile"])

@router.get("/")
async def get_profile(current_user: dict = Depends(get_current_user)):
    user_id = current_user["user_id"]
    # Fetch profile from database using user_id
    ...
```

## Step 4: Database Connection (Backend)

### 4.1 Create Database Service

Update `backend/src/services/database.py`:
```python
import os
import psycopg2
from contextlib import contextmanager

DATABASE_URL = os.getenv("DATABASE_URL")

@contextmanager
def get_db_connection():
    conn = psycopg2.connect(DATABASE_URL, connect_timeout=10)
    try:
        yield conn
    finally:
        conn.close()
```

### 4.2 Add Health Check

Add to `backend/main.py`:
```python
@app.get("/health")
async def health_check():
    try:
        with get_db_connection() as conn:
            with conn.cursor() as cur:
                cur.execute("SELECT 1")
        return {"status": "ok", "database": "connected"}
    except Exception as e:
        return {"status": "error", "database": "disconnected", "detail": str(e)}
```

## Step 5: Run Migration Script

Create `scripts/migrate_supabase_to_neon.py`:
```python
# See research.md for full migration script
# This migrates user profiles from Supabase to Neon
```

## Verification Checklist

- [ ] Neon database connection works: `GET /health` returns `{"database": "connected"}`
- [ ] Better Auth schema created: Check `user`, `account`, `session` tables in Neon
- [ ] Registration works: Create account via frontend
- [ ] Login works: Get session token
- [ ] JWT token issued: Call `/api/auth/token`
- [ ] FastAPI validates JWT: `GET /api/profile` with Bearer token returns profile
- [ ] Profile CRUD works: Create, read, update profile

## Common Issues

| Issue | Solution |
|-------|----------|
| SSL connection error | Ensure `?sslmode=require` in connection string |
| JWKS fetch fails | Check `JWKS_URL` env var, ensure Better Auth server running |
| 401 on all requests | Verify JWT issuer/audience match between frontend and backend |
| Profile not found | Ensure `user_id` FK points to correct Better Auth user |
