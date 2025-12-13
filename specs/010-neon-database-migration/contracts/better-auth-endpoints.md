# Better Auth API Endpoints (Frontend)

**Host**: Docusaurus site (e.g., `https://learn.example.com`)
**Base Path**: `/api/auth`

These endpoints are provided by Better Auth and hosted alongside the Docusaurus frontend.

## Authentication Endpoints

### POST /api/auth/sign-up/email

Create a new user account with email/password.

**Request Body**:
```json
{
  "email": "user@example.com",
  "password": "securePassword123",
  "name": "John Doe"
}
```

**Response (201)**:
```json
{
  "user": {
    "id": "uuid",
    "email": "user@example.com",
    "name": "John Doe",
    "emailVerified": false,
    "createdAt": "2025-12-13T00:00:00Z"
  },
  "session": {
    "id": "uuid",
    "token": "session_token",
    "expiresAt": "2025-12-20T00:00:00Z"
  }
}
```

---

### POST /api/auth/sign-in/email

Authenticate with email/password.

**Request Body**:
```json
{
  "email": "user@example.com",
  "password": "securePassword123"
}
```

**Response (200)**:
```json
{
  "user": {
    "id": "uuid",
    "email": "user@example.com",
    "name": "John Doe"
  },
  "session": {
    "id": "uuid",
    "token": "session_token",
    "expiresAt": "2025-12-20T00:00:00Z"
  }
}
```

**Error (401)**:
```json
{
  "error": "Invalid credentials"
}
```

---

### POST /api/auth/sign-out

Invalidate current session.

**Headers**:
- `Authorization: Bearer <session_token>` or session cookie

**Response (200)**:
```json
{
  "success": true
}
```

---

### GET /api/auth/session

Get current session info.

**Headers**:
- `Authorization: Bearer <session_token>` or session cookie

**Response (200)**:
```json
{
  "user": {
    "id": "uuid",
    "email": "user@example.com",
    "name": "John Doe"
  },
  "session": {
    "id": "uuid",
    "expiresAt": "2025-12-20T00:00:00Z"
  }
}
```

---

## JWT Endpoints

### GET /api/auth/token

Get JWT token for API authentication.

**Headers**:
- `Authorization: Bearer <session_token>` or session cookie

**Response (200)**:
```json
{
  "token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9..."
}
```

**JWT Claims**:
```json
{
  "sub": "user_uuid",
  "iss": "https://learn.example.com",
  "aud": "https://learn.example.com",
  "iat": 1702425600,
  "exp": 1703030400,
  "email": "user@example.com",
  "name": "John Doe"
}
```

---

### GET /api/auth/jwks

Get JSON Web Key Set for JWT verification.

**Response (200)**:
```json
{
  "keys": [
    {
      "kty": "RSA",
      "kid": "key-id",
      "use": "sig",
      "alg": "RS256",
      "n": "...",
      "e": "AQAB"
    }
  ]
}
```

**Usage (FastAPI)**:
```python
from jwt import PyJWKClient

jwks_client = PyJWKClient("https://learn.example.com/api/auth/jwks")
signing_key = jwks_client.get_signing_key_from_jwt(token)
```

---

## Flow Summary

```
1. User registers/logs in via Better Auth
   POST /api/auth/sign-up/email  OR  POST /api/auth/sign-in/email

2. User gets JWT for API calls
   GET /api/auth/token → returns JWT

3. User calls FastAPI with JWT
   GET /api/profile
   Authorization: Bearer <jwt_token>

4. FastAPI validates JWT
   - Fetches JWKS from /api/auth/jwks
   - Verifies signature, issuer, audience, expiration
   - Extracts user_id from 'sub' claim

5. FastAPI queries Neon database for user profile
```
