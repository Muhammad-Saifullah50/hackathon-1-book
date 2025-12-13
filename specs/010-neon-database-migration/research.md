# Research: Neon Database Migration with Better Auth

**Date**: 2025-12-13
**Feature**: 010-neon-database-migration

## Research Areas

### 1. Neon Database Connection (Python/psycopg2)

**Decision**: Use `psycopg2-binary` with connection string from environment variable

**Rationale**:
- Neon officially supports psycopg2 for Python connections
- SSL/TLS is required and handled via `sslmode=require` in connection string
- Connection pooling is managed at Neon's serverless layer
- Simple, well-documented, production-ready

**Alternatives Considered**:
| Alternative | Why Rejected |
|-------------|--------------|
| asyncpg | More complex, async-first; overkill for initial migration |
| SQLAlchemy ORM | Adds abstraction layer; direct psycopg2 simpler for this use case |
| Prisma (Python) | Limited Python support; better suited for Node.js |

**Implementation Pattern**:
```python
import os
import psycopg2
from dotenv import load_dotenv

load_dotenv()
database_url = os.getenv('DATABASE_URL')

conn = psycopg2.connect(database_url, connect_timeout=10)
with conn.cursor() as cur:
    cur.execute("SELECT version()")
    print(cur.fetchone())
conn.close()
```

**Key Configuration**:
- Connection string format: `postgresql://user:password@host.neon.tech/dbname?sslmode=require`
- Connection timeout: 10 seconds (recommended for serverless)
- SSL mode: `require` (mandatory for Neon)

---

### 2. JWT Validation in FastAPI (PyJWT with JWKS)

**Decision**: Use `PyJWT` with `PyJWKClient` for RS256 token validation via JWKS endpoint

**Rationale**:
- PyJWT is the most popular Python JWT library (93.8 benchmark score)
- Native support for JWKS (JSON Web Key Set) fetching
- Supports RS256 algorithm required by Better Auth
- Clean API for audience/issuer validation

**Alternatives Considered**:
| Alternative | Why Rejected |
|-------------|--------------|
| python-jose | Less active maintenance; PyJWT has better JWKS support |
| authlib/joserfc | Heavier dependency; PyJWT sufficient for our needs |
| Manual JWT parsing | Security risk; use well-tested library |

**Implementation Pattern**:
```python
import jwt
from jwt import PyJWKClient

JWKS_URL = "https://your-docusaurus-site.com/api/auth/jwks"
ISSUER = "https://your-docusaurus-site.com"
AUDIENCE = "https://your-docusaurus-site.com"

# Cache the client for performance
jwks_client = PyJWKClient(JWKS_URL)

def validate_token(token: str) -> dict:
    signing_key = jwks_client.get_signing_key_from_jwt(token)
    payload = jwt.decode(
        token,
        signing_key.key,
        algorithms=["RS256"],
        audience=AUDIENCE,
        issuer=ISSUER,
    )
    return payload
```

**Key Configuration**:
- Algorithm: RS256 (asymmetric, public key verification)
- JWKS caching: PyJWKClient handles caching internally
- Expiration: 7 days (configured in Better Auth)

---

### 3. Better Auth Database Schema

**Decision**: Use Better Auth CLI to generate and migrate schema to Neon

**Rationale**:
- Better Auth provides CLI tools for schema generation
- Schema is compatible with PostgreSQL (Neon)
- Handles user, account, session, verification tables automatically

**Schema Tables** (Better Auth Core):
| Table | Purpose |
|-------|---------|
| `user` | User identity (id, email, emailVerified, name, image, timestamps) |
| `account` | Auth provider linking (userId, providerId, accountId, password hash) |
| `session` | Active sessions (userId, token, expiresAt, ipAddress, userAgent) |
| `verification` | Email verification tokens |

**Migration Commands**:
```bash
# Generate schema
npx @better-auth/cli generate

# Apply migration to Neon
npx @better-auth/cli migrate
```

---

### 4. Better Auth with Docusaurus Integration

**Decision**: Integrate Better Auth via Docusaurus swizzling (`src/theme/Root.js`) with companion Node.js server for API routes

**Rationale**:
- Docusaurus is a static site generator; needs server for auth API
- Better Auth client integrates naturally with React
- Root component persists across navigation (ideal for auth state)

**Architecture Options**:
| Option | Chosen | Notes |
|--------|--------|-------|
| Docusaurus + Companion Node.js server | ✅ | Better Auth API runs separately |
| Docusaurus + Serverless (Vercel/Cloudflare) | Alternative | Viable but more complex setup |
| Docusaurus SSR | ❌ | Docusaurus 3.x has limited SSR support |

**Implementation Pattern** (Better Auth Server):
```typescript
// api/auth/[...all].ts (companion server)
import { betterAuth } from "better-auth";
import { Pool } from "pg";

export const auth = betterAuth({
  database: new Pool({
    connectionString: process.env.DATABASE_URL,
  }),
  emailAndPassword: {
    enabled: true,
  },
  jwt: {
    expiresIn: "7d",
  },
});

// Express/Hono handler
app.all("/api/auth/*", (c) => auth.handler(c.req.raw));
```

**Implementation Pattern** (Docusaurus Client):
```typescript
// src/lib/auth.ts
import { createAuthClient } from "better-auth/client";
import { jwtClient } from "better-auth/client/plugins";

export const authClient = createAuthClient({
  baseURL: process.env.AUTH_API_URL,
  plugins: [jwtClient()],
});
```

---

### 5. Supabase to Neon Data Migration

**Decision**: Create migration script using Better Auth's adapter pattern

**Rationale**:
- Better Auth provides migration guide from Supabase
- Script connects to both databases and transfers data
- Handles user, account transformation

**Migration Strategy**:
1. Export profiles from Supabase (existing app data)
2. Create Better Auth schema in Neon
3. Migrate users: Supabase `auth.users` → Better Auth `user` table
4. Migrate accounts: Supabase `auth.identities` → Better Auth `account` table
5. Migrate profiles: Direct copy with FK update to new user IDs

**Environment Variables**:
```dotenv
FROM_DATABASE_URL=postgresql://...@supabase.co/postgres
TO_DATABASE_URL=postgresql://...@neon.tech/neondb
```

---

### 6. CORS Configuration

**Decision**: Configure FastAPI CORS middleware to allow Docusaurus frontend origin

**Rationale**:
- Frontend (Docusaurus) and backend (FastAPI) are on different origins
- CORS required for browser security
- Credentials must be allowed for cookie-based session backup

**Implementation Pattern**:
```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://your-docusaurus-site.com", "http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

---

## Dependencies Summary

### Backend (Python)
```txt
# requirements.txt additions
psycopg2-binary>=2.9.9
PyJWT[crypto]>=2.8.0
python-dotenv>=1.0.0
```

### Frontend (Node.js)
```json
// package.json additions
{
  "dependencies": {
    "better-auth": "^1.0.0",
    "@better-auth/cli": "^1.0.0"
  }
}
```

---

## Open Questions (Resolved)

| Question | Resolution |
|----------|------------|
| Where to host Better Auth API? | Co-located with Docusaurus (companion server) |
| User-Profile relationship? | FK: UserProfile.user_id → User.id (1:1) |
| JWT expiration? | 7 days with session refresh |
| Database driver? | psycopg2-binary (simpler, well-supported) |
| JWT library? | PyJWT with PyJWKClient for JWKS |
