# Auth Server Deployment Guide

## Production Deployment Issue: Session Persistence

### Problem
Users can login/signup successfully, but sessions are not persisted across page reloads in production. The frontend shows login buttons even after successful authentication.

### Root Cause
Cross-domain cookie configuration missing for production deployment. The auth server (`robotook-auth.vercel.app`) and frontend (`robotook.vercel.app`) are on different subdomains, requiring `SameSite=None` and `Secure=true` cookie attributes.

### Solution Applied

#### 1. Cookie Configuration (`auth-server/src/auth.ts`)
Added `advanced.cookies` configuration:
```typescript
advanced: {
  cookies: {
    sessionToken: {
      name: "better-auth.session_token",
      attributes: {
        sameSite: "none", // Required for cross-domain
        secure: true,     // Required with SameSite=None
        httpOnly: true,
        path: "/",
        domain: process.env.COOKIE_DOMAIN || undefined,
      },
    },
  },
}
```

#### 2. Trusted Origins
Updated to explicitly include production domains:
```typescript
trustedOrigins: [
  "http://localhost:3000",
  "http://localhost:8000",
  "https://robotook.vercel.app",
  "https://robotook-auth.vercel.app",
]
```

#### 3. CORS Configuration (`auth-server/src/index.ts`)
Matched CORS origins with trusted origins.

## Vercel Environment Variables

Set these in Vercel dashboard for `robotook-auth` project:

```bash
# Database
DATABASE_URL=postgresql://...  # Neon connection string

# Auth URLs
BETTER_AUTH_URL=https://robotook-auth.vercel.app

# CORS (optional - already hardcoded in code)
CORS_ORIGINS=https://robotook.vercel.app

# Cookie domain (optional - only if sharing cookies across subdomains)
# COOKIE_DOMAIN=.vercel.app
```

## Deployment Steps

### 1. Deploy Auth Server
```bash
cd auth-server
vercel --prod
```

### 2. Verify Deployment
```bash
# Check health endpoint
curl https://robotook-auth.vercel.app/health

# Check JWKS endpoint
curl https://robotook-auth.vercel.app/api/auth/jwks

# Test session endpoint
curl -X GET https://robotook-auth.vercel.app/api/auth/get-session \
  -H "Content-Type: application/json" \
  --cookie "better-auth.session_token=YOUR_TOKEN"
```

### 3. Test Authentication Flow

1. **Signup**: Visit `https://robotook.vercel.app/signup`
2. **Login**: Use credentials to login
3. **Check Browser DevTools**:
   - **Application > Cookies**: Verify `better-auth.session_token` exists with:
     - `SameSite: None`
     - `Secure: true`
     - `HttpOnly: true`
   - **Network > XHR**: Verify `/api/auth/get-session` returns 200 OK
4. **Refresh Page**: User should remain logged in

## Troubleshooting

### Issue: Session not persisted after refresh
**Check**:
1. Cookie attributes in browser DevTools (must have `SameSite=None; Secure`)
2. CORS origins match in both auth-server config and Vercel env vars
3. `BETTER_AUTH_URL` environment variable is set correctly

### Issue: CORS errors
**Check**:
1. Frontend origin is in `trustedOrigins` array
2. `credentials: true` is set in CORS config
3. Backend is returning `Access-Control-Allow-Credentials: true` header

### Issue: 404 on `/api/auth/get-session`
**Check**:
1. Auth server is deployed and running
2. `toNodeHandler(auth)` route is mounted at `/api/auth/*`
3. Vercel routing configuration is correct

## Architecture

```
┌──────────────────────────────────────┐
│  Frontend (robotook.vercel.app)     │
│  - Docusaurus static site            │
│  - Better Auth React client          │
└────────────┬─────────────────────────┘
             │
             │ (HTTPS + cookies)
             │ SameSite=None; Secure
             │
┌────────────▼─────────────────────────┐
│  Auth Server (robotook-auth.v...)   │
│  - Express + Better Auth             │
│  - Neon PostgreSQL                   │
│  - JWT + Session management          │
└────────────┬─────────────────────────┘
             │
             │ (SQL)
             │
┌────────────▼─────────────────────────┐
│  Neon PostgreSQL                     │
│  - user, session, account tables     │
└──────────────────────────────────────┘
```

## Security Notes

- **SameSite=None**: Required for cross-domain cookies, but increases CSRF risk. Mitigated by:
  - `Secure=true` (HTTPS only)
  - `HttpOnly=true` (no JavaScript access)
  - CORS origin validation
  - Better Auth's built-in CSRF protection

- **Cookie Domain**: NOT setting a shared domain (e.g., `.vercel.app`) for security. Each subdomain maintains its own cookie scope.

## Local Development

For local testing with cross-domain setup:

```bash
# Terminal 1: Auth server
cd auth-server
npm run dev

# Terminal 2: Frontend
cd website
npm start

# Terminal 3: Backend API
cd backend
uv run uvicorn main:app --reload
```

Cookies will work locally without `SameSite=None` since all services are on `localhost`.
