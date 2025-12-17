# Auth Server

**Better Auth Authentication Server for Physical AI & Humanoid Robotics Learning Platform**

A secure, standalone Express authentication server powered by Better Auth, providing JWT-based authentication with Neon PostgreSQL storage for a full-stack educational platform.

---

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Features](#features)
- [Technology Stack](#technology-stack)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
  - [Environment Configuration](#environment-configuration)
  - [Database Setup](#database-setup)
- [Development](#development)
- [API Reference](#api-reference)
- [Security Considerations](#security-considerations)
- [Production Deployment](#production-deployment)
- [Troubleshooting](#troubleshooting)
- [Project Structure](#project-structure)

---

## Overview

This authentication server acts as the central identity provider for the Physical AI & Humanoid Robotics Learning Platform. It handles user registration, authentication, session management, and JWT token issuance using Better Auth - a modern, TypeScript-first authentication framework.

**Key Responsibilities:**
- User registration with email/password
- User authentication and session management
- JWT token issuance for API authorization
- JWKS endpoint for public key distribution
- Secure cookie handling for session persistence

**Integration Points:**
- **Frontend (Docusaurus)**: Users authenticate via Better Auth React client
- **Backend (FastAPI)**: Validates JWT tokens using JWKS for API requests
- **Database (Neon PostgreSQL)**: Stores users, sessions, and accounts

---

## Architecture

```
┌─────────────────────┐         ┌─────────────────────┐         ┌─────────────────────┐
│   Docusaurus        │         │   Auth Server       │         │   FastAPI Backend   │
│   (Frontend)        │────────>│   (This Service)    │         │   (API)             │
│   Port: 3000        │  Auth   │   Port: 3001        │         │   Port: 8000        │
│                     │  Flow   │                     │         │                     │
│  Better Auth Client │         │  Better Auth Server │         │  JWT Validation     │
│  - signup()         │         │  - emailAndPassword │         │  - JWKS fetch       │
│  - signIn()         │         │  - JWT plugin       │         │  - Token verify     │
│  - signOut()        │         │  - Session mgmt     │         │  - User extraction  │
│  - useSession()     │         │  - Cookie handling  │         │                     │
└─────────────────────┘         └──────────┬──────────┘         └──────────┬──────────┘
                                           │                               │
                                           │         ┌────────────────────┘
                                           │         │
                                           ▼         ▼
                                  ┌─────────────────────┐
                                  │  Neon PostgreSQL    │
                                  │  (Serverless)       │
                                  │                     │
                                  │  Tables:            │
                                  │  - user             │
                                  │  - session          │
                                  │  - account          │
                                  │  - verification     │
                                  └─────────────────────┘
```

**Authentication Flow:**

1. **User Registration/Login** → Frontend calls `/api/auth/sign-up/email` or `/api/auth/sign-in/email`
2. **Session Creation** → Auth server creates session, stores in Neon, returns secure cookie
3. **JWT Issuance** → Frontend requests JWT via `/api/auth/token` for API calls
4. **API Authorization** → FastAPI validates JWT using public keys from `/api/auth/jwks`
5. **Session Refresh** → Sessions auto-refresh every 24 hours, expire after 7 days

---

## Features

- **Email/Password Authentication**: Secure user registration and login with bcrypt hashing
- **JWT Token Issuance**: RS256-signed JWT tokens for stateless API authentication
- **JWKS Endpoint**: Public key distribution for distributed JWT verification
- **Session Management**: 7-day sessions with automatic refresh
- **Secure Cookie Handling**: HttpOnly, Secure, SameSite cookies for production
- **CORS Configuration**: Flexible cross-origin resource sharing for multiple frontends
- **Connection Pooling**: Optimized Neon PostgreSQL connections with automatic idle cleanup
- **Health Monitoring**: Built-in health check endpoint for deployment monitoring
- **TypeScript**: Full type safety across the entire codebase

---

## Technology Stack

| Technology | Version | Purpose |
|------------|---------|---------|
| **Node.js** | >= 20.0 | JavaScript runtime |
| **TypeScript** | 5.7.2 | Type-safe development |
| **Express** | 4.21.2 | Web server framework |
| **Better Auth** | 1.2.8 | Authentication framework |
| **PostgreSQL (Neon)** | - | Serverless database |
| **node-postgres (pg)** | 8.13.1 | PostgreSQL client |
| **CORS** | 2.8.5 | Cross-origin requests |
| **dotenv** | 16.4.7 | Environment variables |
| **tsx** | 4.19.2 | TypeScript execution (dev) |
| **Vitest** | 2.1.8 | Unit testing framework |

---

## Getting Started

### Prerequisites

Ensure you have the following installed:

- **Node.js** >= 20.0 ([Download](https://nodejs.org/))
- **npm** or **pnpm** package manager
- **Neon PostgreSQL** database ([Sign up](https://neon.tech/))
- **Git** for version control

### Installation

1. **Clone the repository and navigate to auth-server:**
   ```bash
   cd auth-server
   ```

2. **Install dependencies:**
   ```bash
   npm install
   ```

3. **Verify installation:**
   ```bash
   npm run build
   ```

### Environment Configuration

Create a `.env` file in the `auth-server/` directory:

```bash
# Server Configuration
PORT=3001
NODE_ENV=development

# Neon Database Connection
# Get this from your Neon dashboard: https://console.neon.tech
DATABASE_URL=postgresql://username:password@ep-xxx.us-east-2.aws.neon.tech/dbname?sslmode=require

# Better Auth Configuration
# Generate a random 32+ character secret: openssl rand -base64 32
BETTER_AUTH_SECRET=your-secret-key-at-least-32-characters-long-change-this-in-production
BETTER_AUTH_URL=http://localhost:3001

# CORS Origins (comma-separated, no spaces)
CORS_ORIGINS=http://localhost:3000,http://localhost:8000

# JWT Configuration (optional - defaults shown)
JWT_EXPIRES_IN=7d
```

**Critical Security Notes:**
- Never commit `.env` to version control
- Use a strong, unique `BETTER_AUTH_SECRET` (minimum 32 characters)
- Generate secret with: `openssl rand -base64 32`
- Change default secrets before deploying to production

### Database Setup

Better Auth requires specific database tables. Run migrations to create them:

1. **Generate Better Auth schema:**
   ```bash
   npm run generate
   ```
   This creates a `schema.sql` file with the required table definitions.

2. **Apply Better Auth migration:**
   ```bash
   npm run migrate
   ```
   This automatically creates `user`, `session`, `account`, and `verification` tables in your Neon database.

3. **Verify tables were created:**
   ```bash
   psql $DATABASE_URL -c "\dt"
   ```
   You should see:
   - `user` - User accounts
   - `session` - Active sessions
   - `account` - OAuth accounts (if using social auth)
   - `verification` - Email verification tokens

**Schema Overview:**

```sql
-- user table
CREATE TABLE user (
  id TEXT PRIMARY KEY,
  email TEXT UNIQUE NOT NULL,
  email_verified BOOLEAN DEFAULT false,
  name TEXT,
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW()
);

-- session table
CREATE TABLE session (
  id TEXT PRIMARY KEY,
  expires_at TIMESTAMP NOT NULL,
  token TEXT UNIQUE NOT NULL,
  user_id TEXT REFERENCES user(id) ON DELETE CASCADE,
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW()
);
```

---

## Development

### Start Development Server

```bash
npm run dev
```

Server starts at `http://localhost:3001` with hot-reload enabled.

**Verify it's running:**
```bash
curl http://localhost:3001/health
```

**Expected response:**
```json
{
  "status": "ok",
  "service": "auth-server",
  "timestamp": "2025-12-17T20:00:00.000Z"
}
```

### Development Commands

| Command | Description |
|---------|-------------|
| `npm run dev` | Start development server with hot reload (tsx) |
| `npm run build` | Compile TypeScript to JavaScript |
| `npm start` | Start production server (requires build) |
| `npm run generate` | Generate Better Auth schema SQL |
| `npm run migrate` | Apply migrations to database |
| `npm test` | Run unit tests with Vitest |

### Project File Structure

```
auth-server/
├── src/
│   ├── index.ts          # Express app & server entry point
│   ├── auth.ts           # Better Auth configuration
│   └── db.ts             # Neon PostgreSQL connection pool
├── tests/                # Unit tests
├── dist/                 # Compiled JavaScript (generated)
├── node_modules/         # Dependencies
├── .env                  # Environment variables (not committed)
├── package.json          # Project metadata & scripts
├── tsconfig.json         # TypeScript configuration
├── vercel.json           # Vercel deployment config
└── README.md            # This file
```

### Local Development Workflow

**Terminal 1 - Auth Server:**
```bash
cd auth-server
npm run dev
# Running on http://localhost:3001
```

**Terminal 2 - Frontend:**
```bash
cd website
npm start
# Running on http://localhost:3000
```

**Terminal 3 - Backend API:**
```bash
cd backend
uv run uvicorn main:app --reload
# Running on http://localhost:8000
```

**Test the full auth flow:**
1. Open `http://localhost:3000`
2. Click "Sign Up" and create an account
3. Check auth-server logs for session creation
4. Verify cookie is set in browser DevTools

---

## API Reference

All Better Auth endpoints are automatically mounted at `/api/auth/*` by the server.

### Authentication Endpoints

#### POST `/api/auth/sign-up/email`

Register a new user account.

**Request:**
```bash
curl -X POST http://localhost:3001/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{
    "email": "user@example.com",
    "password": "SecurePassword123!",
    "name": "John Doe"
  }'
```

**Response (201 Created):**
```json
{
  "user": {
    "id": "cm4x5y6z7a8b9c0d1e2f3g4h",
    "email": "user@example.com",
    "name": "John Doe",
    "emailVerified": false,
    "createdAt": "2025-12-17T20:00:00.000Z",
    "updatedAt": "2025-12-17T20:00:00.000Z"
  },
  "session": {
    "id": "sess_abc123def456",
    "token": "session_token_here",
    "expiresAt": "2025-12-24T20:00:00.000Z"
  }
}
```

**Password Requirements:**
- Minimum 8 characters
- Maximum 128 characters
- Automatically hashed with bcrypt

---

#### POST `/api/auth/sign-in/email`

Authenticate with email and password.

**Request:**
```bash
curl -X POST http://localhost:3001/api/auth/sign-in/email \
  -H "Content-Type: application/json" \
  -d '{
    "email": "user@example.com",
    "password": "SecurePassword123!"
  }'
```

**Response (200 OK):**
```json
{
  "user": {
    "id": "cm4x5y6z7a8b9c0d1e2f3g4h",
    "email": "user@example.com",
    "name": "John Doe"
  },
  "session": {
    "id": "sess_abc123def456",
    "token": "session_token_here",
    "expiresAt": "2025-12-24T20:00:00.000Z"
  }
}
```

**Error Response (401 Unauthorized):**
```json
{
  "error": "Invalid credentials",
  "message": "Email or password is incorrect"
}
```

---

#### POST `/api/auth/sign-out`

Invalidate the current session.

**Request:**
```bash
curl -X POST http://localhost:3001/api/auth/sign-out \
  -H "Cookie: better-auth.session_token=YOUR_TOKEN"
```

**Response (200 OK):**
```json
{
  "success": true
}
```

---

#### GET `/api/auth/session`

Get current session information.

**Request:**
```bash
curl -X GET http://localhost:3001/api/auth/session \
  -H "Cookie: better-auth.session_token=YOUR_TOKEN"
```

**Response (200 OK):**
```json
{
  "user": {
    "id": "cm4x5y6z7a8b9c0d1e2f3g4h",
    "email": "user@example.com",
    "name": "John Doe"
  },
  "session": {
    "id": "sess_abc123def456",
    "expiresAt": "2025-12-24T20:00:00.000Z"
  }
}
```

**Error Response (401 Unauthorized):**
```json
{
  "error": "No active session",
  "message": "Session token is missing or expired"
}
```

---

### JWT Endpoints

#### POST `/api/auth/token`

Get a JWT token for API authentication.

**Request:**
```bash
curl -X POST http://localhost:3001/api/auth/token \
  -H "Cookie: better-auth.session_token=YOUR_SESSION_TOKEN"
```

**Response (200 OK):**
```json
{
  "token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCIsImtpZCI6ImtleS1pZC0xMjM0In0.eyJzdWIiOiJjbTR4NXk2ejdhOGI5YzBkMWUyZjNnNGgiLCJlbWFpbCI6InVzZXJAZXhhbXBsZS5jb20iLCJuYW1lIjoiSm9obiBEb2UiLCJpc3MiOiJodHRwOi8vbG9jYWxob3N0OjMwMDEiLCJhdWQiOiJodHRwOi8vbG9jYWxob3N0OjMwMDEiLCJpYXQiOjE3MDI0MjU2MDAsImV4cCI6MTcwMzAzMDQwMH0.signature..."
}
```

**JWT Claims:**
```json
{
  "sub": "cm4x5y6z7a8b9c0d1e2f3g4h",  // User ID
  "email": "user@example.com",
  "name": "John Doe",
  "iss": "http://localhost:3001",      // Issuer
  "aud": "http://localhost:3001",      // Audience
  "iat": 1702425600,                    // Issued at (Unix timestamp)
  "exp": 1703030400                     // Expires at (Unix timestamp)
}
```

**Usage in API calls:**
```bash
curl -X GET http://localhost:8000/api/profile \
  -H "Authorization: Bearer YOUR_JWT_TOKEN"
```

---

#### GET `/api/auth/jwks`

Get JSON Web Key Set for JWT verification.

**Request:**
```bash
curl http://localhost:3001/api/auth/jwks
```

**Response (200 OK):**
```json
{
  "keys": [
    {
      "kty": "RSA",
      "kid": "key-id-1234",
      "use": "sig",
      "alg": "RS256",
      "n": "xGOr-H7A4QvYPZxKq_long_modulus_here...",
      "e": "AQAB"
    }
  ]
}
```

**Purpose:**
- Allows FastAPI backend to verify JWT signatures without shared secrets
- Public keys rotate automatically for security
- Follows [RFC 7517 (JWK)](https://datatracker.ietf.org/doc/html/rfc7517) standard

**FastAPI Integration Example:**
```python
from jose import jwt
from jose.backends import RSABackend
import requests

# Fetch JWKS
jwks_url = "http://localhost:3001/api/auth/jwks"
jwks = requests.get(jwks_url).json()

# Verify JWT
token = "eyJhbGciOiJSUzI1NiI..."
payload = jwt.decode(
    token,
    jwks,
    algorithms=["RS256"],
    audience="http://localhost:3001"
)
user_id = payload["sub"]
```

---

### Health Check

#### GET `/health`

Check server status and uptime.

**Request:**
```bash
curl http://localhost:3001/health
```

**Response (200 OK):**
```json
{
  "status": "ok",
  "service": "auth-server",
  "timestamp": "2025-12-17T20:00:00.000Z"
}
```

**Use cases:**
- Deployment health checks (Vercel, Railway, etc.)
- Load balancer health probes
- Monitoring and alerting systems

---

## Security Considerations

### Authentication Security

1. **Password Hashing**
   - Passwords are hashed with bcrypt (cost factor: 10)
   - Salting is automatic and unique per password
   - Never stored or logged in plaintext

2. **JWT Security**
   - Signed with RS256 (RSA + SHA-256) algorithm
   - Private key is auto-generated and stored securely
   - 7-day expiration enforced (configurable)
   - Includes issuer (`iss`) and audience (`aud`) claims for validation

3. **Session Management**
   - Sessions expire after 7 days of inactivity
   - Auto-refresh every 24 hours during active use
   - Secure session tokens stored in HttpOnly cookies
   - Sessions invalidated on logout or password change

### Cookie Security

**Development (localhost):**
```typescript
{
  sameSite: "lax",
  secure: false,      // HTTP allowed
  httpOnly: true,
  path: "/"
}
```

**Production (HTTPS):**
```typescript
{
  sameSite: "lax",    // Same-origin requests only
  secure: true,       // HTTPS required
  httpOnly: true,     // No JavaScript access
  path: "/"
}
```

**Cookie Attributes Explained:**
- `httpOnly: true` - Prevents XSS attacks (JavaScript cannot read cookie)
- `secure: true` - HTTPS-only transmission (prevents MITM attacks)
- `sameSite: "lax"` - Protects against CSRF attacks (no cross-site POST)
- `path: "/"` - Cookie available to all paths on the domain

**Important Notes:**
- **Same-Origin Policy**: In production, auth-server must be on the same origin as the frontend (via Vercel rewrites or reverse proxy)
- **No Shared Domains**: `.vercel.app` is a public suffix and cannot be used as a shared cookie domain
- **Cross-Domain Cookies**: If you absolutely need cross-domain cookies, use `sameSite: "none"` + `secure: true`, but this increases CSRF risk

### Database Security

1. **Connection Security**
   - SSL/TLS required for all Neon connections
   - Connection string stored in environment variables only
   - Connection pooling limits: 10 max, 30s idle timeout

2. **SQL Injection Prevention**
   - All queries use parameterized statements (pg library)
   - Better Auth handles query sanitization automatically

3. **Data Protection**
   - User emails are unique and indexed
   - Passwords never stored in plaintext
   - Sessions are purged on expiration

### CORS Security

**Allowed Origins (configurable via env vars):**
```typescript
const corsOrigins = [
  "http://localhost:3000",        // Frontend (dev)
  "http://localhost:8000",        // Backend API (dev)
  "https://robotook.vercel.app",  // Frontend (prod)
  ...process.env.CORS_ORIGINS.split(",")
];
```

**CORS Configuration:**
```typescript
cors({
  origin: corsOrigins,
  credentials: true,  // Allow cookies
  methods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
  allowedHeaders: ["Content-Type", "Authorization"]
})
```

**Best Practices:**
- Whitelist specific origins (never use `*` with credentials)
- Keep CORS_ORIGINS environment variable minimal
- Use HTTPS in production

### Environment Variables Security

**Secrets Management:**
- Never commit `.env` files to version control
- Use platform-specific secret managers in production:
  - Vercel: Environment Variables in dashboard
  - Railway: Variables tab in project settings
  - AWS: Secrets Manager or Parameter Store
- Rotate `BETTER_AUTH_SECRET` if compromised
- Use different secrets for dev/staging/production

**Minimum Secret Strength:**
```bash
# Generate a secure secret (32+ characters)
openssl rand -base64 32
```

### Rate Limiting (Recommended)

While not implemented by default, consider adding rate limiting for production:

```typescript
import rateLimit from "express-rate-limit";

const authLimiter = rateLimit({
  windowMs: 15 * 60 * 1000, // 15 minutes
  max: 5, // 5 requests per window
  message: "Too many login attempts, please try again later"
});

app.post("/api/auth/sign-in/email", authLimiter, ...);
```

---

## Production Deployment

### Recommended Platforms

| Platform | Pros | Setup Difficulty |
|----------|------|------------------|
| **Vercel** | Auto-deploy, edge network, free tier | Easy |
| **Railway** | Simple, built-in PostgreSQL, affordable | Easy |
| **Render** | Free tier, managed databases | Easy |
| **Fly.io** | Global edge network, Docker-based | Medium |
| **AWS ECS/Fargate** | Full control, scalable | Hard |

### Vercel Deployment

1. **Install Vercel CLI:**
   ```bash
   npm install -g vercel
   ```

2. **Deploy:**
   ```bash
   cd auth-server
   vercel --prod
   ```

3. **Set Environment Variables** in Vercel Dashboard:
   ```bash
   DATABASE_URL=postgresql://...
   BETTER_AUTH_SECRET=<strong-random-secret>
   BETTER_AUTH_URL=https://your-auth-server.vercel.app
   CORS_ORIGINS=https://your-frontend.vercel.app
   ```

4. **Configure Same-Origin Deployment:**

   To avoid cross-domain cookie issues, use Vercel rewrites in your **frontend** `vercel.json`:

   ```json
   {
     "rewrites": [
       {
         "source": "/api/auth/:path*",
         "destination": "https://your-auth-server.vercel.app/api/auth/:path*"
       }
     ]
   }
   ```

   This makes auth endpoints appear on the same origin as your frontend.

### Railway Deployment

1. **Create Railway Project:**
   ```bash
   railway login
   railway init
   ```

2. **Add Neon PostgreSQL:**
   - Link your existing Neon database via `DATABASE_URL` environment variable

3. **Deploy:**
   ```bash
   railway up
   ```

4. **Configure Environment Variables** in Railway dashboard

### Environment Variables for Production

```bash
# Server
NODE_ENV=production
PORT=3001  # Railway/Render may override this

# Database
DATABASE_URL=postgresql://user:pass@ep-xxx.aws.neon.tech/db?sslmode=require

# Auth
BETTER_AUTH_SECRET=<64-character-random-string>
BETTER_AUTH_URL=https://auth.yourdomain.com

# CORS
CORS_ORIGINS=https://yourdomain.com,https://api.yourdomain.com
```

### Post-Deployment Verification

1. **Health Check:**
   ```bash
   curl https://your-auth-server.vercel.app/health
   ```

2. **JWKS Endpoint:**
   ```bash
   curl https://your-auth-server.vercel.app/api/auth/jwks
   ```

3. **Test Registration:**
   ```bash
   curl -X POST https://your-auth-server.vercel.app/api/auth/sign-up/email \
     -H "Content-Type: application/json" \
     -d '{"email":"test@example.com","password":"TestPass123!","name":"Test User"}'
   ```

4. **Test Login:**
   ```bash
   curl -X POST https://your-auth-server.vercel.app/api/auth/sign-in/email \
     -H "Content-Type: application/json" \
     -d '{"email":"test@example.com","password":"TestPass123!"}'
   ```

### Monitoring and Observability

**Log Aggregation:**
- Vercel: Built-in logs in dashboard
- Railway: Built-in logs with search
- Self-hosted: Use Winston + CloudWatch/Datadog

**Metrics to Monitor:**
- Request rate to `/api/auth/*` endpoints
- Error rate (4xx, 5xx responses)
- Database connection pool health
- JWT verification failures
- Average response time

**Alerting:**
- Set up alerts for 5xx errors
- Monitor database connection failures
- Track unusual login patterns

---

## Troubleshooting

### Common Issues

#### Issue: "Cannot connect to database"

**Symptoms:**
```
Error: connect ECONNREFUSED
```

**Solutions:**
1. Verify `DATABASE_URL` is correct in `.env`
2. Check Neon database is active (free tier auto-pauses after inactivity)
3. Ensure `sslmode=require` is in connection string
4. Test connection manually:
   ```bash
   psql $DATABASE_URL -c "SELECT NOW();"
   ```

---

#### Issue: "Session not persisted after page refresh"

**Symptoms:**
- User logs in successfully
- Page reload shows "Sign In" button again
- No cookie visible in DevTools

**Solutions:**

1. **Check Cookie Attributes:**
   - Open DevTools > Application > Cookies
   - Look for `better-auth.session_token`
   - Verify attributes: `HttpOnly`, `Secure` (production only), `SameSite=Lax`

2. **Verify CORS Configuration:**
   ```typescript
   // auth-server/src/index.ts
   cors({
     origin: "https://your-frontend.vercel.app",
     credentials: true  // MUST be true
   })
   ```

3. **Check Same-Origin Setup:**
   - In production, auth endpoints MUST be on same origin as frontend
   - Use Vercel rewrites or reverse proxy
   - Cannot use cross-domain cookies with public suffixes like `.vercel.app`

4. **Inspect Network Requests:**
   - DevTools > Network > XHR
   - Check `/api/auth/session` response
   - Should return 200 with user data, not 401

---

#### Issue: "CORS errors in browser console"

**Symptoms:**
```
Access to fetch at 'http://localhost:3001/api/auth/sign-in/email'
from origin 'http://localhost:3000' has been blocked by CORS policy
```

**Solutions:**

1. **Add Frontend Origin to CORS:**
   ```bash
   # .env
   CORS_ORIGINS=http://localhost:3000
   ```

2. **Verify CORS Middleware Order:**
   ```typescript
   // CORS must come BEFORE route handlers
   app.use(cors({...}));
   app.all("/api/auth/*", toNodeHandler(auth));
   ```

3. **Check Preflight Requests:**
   - Ensure `OPTIONS` method is allowed
   - CORS config should include: `methods: ["GET", "POST", "OPTIONS"]`

---

#### Issue: "JWT verification fails in FastAPI"

**Symptoms:**
```python
jose.exceptions.JWTError: Signature verification failed
```

**Solutions:**

1. **Verify JWKS URL:**
   ```python
   # Should match auth-server URL
   jwks_url = "https://your-auth-server.vercel.app/api/auth/jwks"
   ```

2. **Check JWT Claims:**
   ```python
   # Audience must match auth-server URL
   jwt.decode(
       token,
       jwks,
       algorithms=["RS256"],
       audience="https://your-auth-server.vercel.app"  # Must match BETTER_AUTH_URL
   )
   ```

3. **Inspect Token:**
   ```bash
   # Decode JWT (without verification)
   echo "YOUR_TOKEN" | base64 -d
   ```

4. **Verify Clock Sync:**
   - JWT expiration depends on server time
   - Ensure auth-server and FastAPI server clocks are synchronized

---

#### Issue: "Database migrations fail"

**Symptoms:**
```
Error: relation "user" already exists
```

**Solutions:**

1. **Drop Existing Tables (DEV ONLY):**
   ```sql
   DROP TABLE IF EXISTS verification, account, session, "user" CASCADE;
   ```

2. **Re-run Migration:**
   ```bash
   npm run generate
   npm run migrate
   ```

3. **Manual Migration:**
   ```bash
   psql $DATABASE_URL -f schema.sql
   ```

---

### Debug Mode

Enable verbose logging for troubleshooting:

```typescript
// auth-server/src/index.ts
app.use((req, res, next) => {
  console.log(`${req.method} ${req.url}`);
  console.log("Headers:", req.headers);
  console.log("Cookies:", req.cookies);
  next();
});
```

**Production Warning:** Disable debug logging in production to avoid leaking sensitive data.

---

### Getting Help

1. **Better Auth Documentation:** [https://better-auth.com/docs](https://better-auth.com/docs)
2. **Better Auth Discord:** [https://discord.gg/better-auth](https://discord.gg/better-auth)
3. **Neon Support:** [https://neon.tech/docs](https://neon.tech/docs)
4. **Project Issues:** [GitHub Issues](https://github.com/your-org/your-repo/issues)

---

## Project Structure

```
auth-server/
├── src/
│   ├── index.ts                # Express server & middleware
│   │   - CORS configuration
│   │   - Health check endpoint
│   │   - Better Auth route handler
│   │
│   ├── auth.ts                 # Better Auth configuration
│   │   - Email/password authentication
│   │   - JWT plugin (RS256)
│   │   - Session settings (7-day expiration)
│   │   - Cookie security (HttpOnly, Secure, SameSite)
│   │
│   └── db.ts                   # Neon PostgreSQL connection
│       - Connection pool (max 10)
│       - SSL configuration
│       - Connection error handling
│
├── tests/                      # Unit tests (Vitest)
│   └── auth.test.ts            # Auth flow tests
│
├── dist/                       # Compiled JavaScript (generated by tsc)
│
├── node_modules/               # Dependencies (npm install)
│
├── .env                        # Environment variables (NOT committed)
│   - DATABASE_URL
│   - BETTER_AUTH_SECRET
│   - BETTER_AUTH_URL
│   - CORS_ORIGINS
│
├── package.json                # Project metadata & scripts
├── tsconfig.json               # TypeScript compiler config
├── vercel.json                 # Vercel deployment config
├── DEPLOYMENT.md               # Production deployment guide
└── README.md                   # This file
```

### Key Files Explained

**`src/index.ts`** - Express server entry point:
- Initializes Express app
- Configures CORS for cross-origin requests
- Mounts Better Auth handler at `/api/auth/*`
- Provides `/health` endpoint for monitoring
- Starts server on specified port

**`src/auth.ts`** - Better Auth configuration:
- Configures email/password authentication (8-128 char passwords)
- Enables JWT plugin with RS256 signing algorithm
- Sets session expiration (7 days) and refresh policy (24 hours)
- Defines cookie attributes (HttpOnly, Secure, SameSite)
- Specifies trusted origins for CORS

**`src/db.ts`** - PostgreSQL connection pool:
- Creates connection pool with Neon PostgreSQL
- Configures SSL (required for Neon)
- Sets pool limits (10 max connections, 30s idle timeout)
- Logs connection events for debugging

**`vercel.json`** - Vercel deployment configuration:
- Specifies build configuration (`@vercel/node`)
- Routes all requests to `src/index.ts`
- Enables serverless function deployment

---

## Integration Guide

### Frontend Integration (React/Docusaurus)

1. **Install Better Auth Client:**
   ```bash
   npm install better-auth
   ```

2. **Create Auth Client:**
   ```typescript
   // website/src/lib/auth-client.ts
   import { createAuthClient } from "better-auth/react";

   export const authClient = createAuthClient({
     baseURL: "http://localhost:3001"  // Auth server URL
   });

   export const { signIn, signUp, signOut, useSession } = authClient;
   ```

3. **Use in Components:**
   ```typescript
   import { signUp, signIn, useSession } from "@/lib/auth-client";

   function LoginForm() {
     const { data: session, isPending } = useSession();

     const handleLogin = async (email: string, password: string) => {
       await signIn.email({ email, password });
     };

     if (isPending) return <div>Loading...</div>;
     if (session) return <div>Welcome, {session.user.name}</div>;

     return <form onSubmit={(e) => {
       e.preventDefault();
       const formData = new FormData(e.currentTarget);
       handleLogin(
         formData.get("email") as string,
         formData.get("password") as string
       );
     }}>
       <input name="email" type="email" required />
       <input name="password" type="password" required />
       <button type="submit">Login</button>
     </form>;
   }
   ```

### Backend Integration (FastAPI)

1. **Install JWT Library:**
   ```bash
   pip install python-jose[cryptography] requests
   ```

2. **Create JWT Validator:**
   ```python
   # backend/src/auth/jwt_validator.py
   from jose import jwt, jwk
   from jose.utils import base64url_decode
   import requests
   from typing import Dict

   class JWTValidator:
       def __init__(self, jwks_url: str, audience: str):
           self.jwks_url = jwks_url
           self.audience = audience
           self._jwks_cache = None

       def get_jwks(self) -> Dict:
           if not self._jwks_cache:
               response = requests.get(self.jwks_url)
               self._jwks_cache = response.json()
           return self._jwks_cache

       def verify_token(self, token: str) -> Dict:
           # Get JWKS
           jwks = self.get_jwks()

           # Decode and verify JWT
           payload = jwt.decode(
               token,
               jwks,
               algorithms=["RS256"],
               audience=self.audience
           )

           return payload

   # Initialize validator
   jwt_validator = JWTValidator(
       jwks_url="http://localhost:3001/api/auth/jwks",
       audience="http://localhost:3001"
   )
   ```

3. **Create Auth Dependency:**
   ```python
   # backend/src/auth/dependencies.py
   from fastapi import Depends, HTTPException, status
   from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
   from .jwt_validator import jwt_validator

   security = HTTPBearer()

   async def get_current_user_id(
       credentials: HTTPAuthorizationCredentials = Depends(security)
   ) -> str:
       try:
           payload = jwt_validator.verify_token(credentials.credentials)
           user_id = payload.get("sub")
           if not user_id:
               raise HTTPException(
                   status_code=status.HTTP_401_UNAUTHORIZED,
                   detail="Invalid token: missing user ID"
               )
           return user_id
       except Exception as e:
           raise HTTPException(
               status_code=status.HTTP_401_UNAUTHORIZED,
               detail=f"Invalid token: {str(e)}"
           )
   ```

4. **Use in API Routes:**
   ```python
   # backend/src/api/profile.py
   from fastapi import APIRouter, Depends
   from ..auth.dependencies import get_current_user_id

   router = APIRouter()

   @router.get("/api/profile")
   async def get_profile(user_id: str = Depends(get_current_user_id)):
       # user_id is extracted from JWT token
       profile = await fetch_profile_from_db(user_id)
       return profile
   ```

---

## License

This auth server is part of the Physical AI & Humanoid Robotics Learning Platform.

**Dependencies:**
- Better Auth: MIT License
- Express: MIT License
- node-postgres: MIT License

---

## Contributing

Contributions are welcome! Please follow these guidelines:

1. **Fork the repository**
2. **Create a feature branch:** `git checkout -b feature/my-feature`
3. **Follow code standards:**
   - TypeScript strict mode enabled
   - Use async/await (no callbacks)
   - Add JSDoc comments for public functions
4. **Write tests:** Add Vitest tests for new features
5. **Test locally:**
   ```bash
   npm run build
   npm test
   npm run dev
   ```
6. **Commit with descriptive messages:**
   ```bash
   git commit -m "feat: add email verification endpoint"
   ```
7. **Submit a pull request**

---

## Support

If you encounter issues or have questions:

1. Check the [Troubleshooting](#troubleshooting) section
2. Review [Better Auth Documentation](https://better-auth.com/docs)
3. Open a [GitHub Issue](https://github.com/your-org/your-repo/issues)
4. Join the [Better Auth Discord](https://discord.gg/better-auth)

---

**Built with Better Auth** - [https://better-auth.com](https://better-auth.com)
