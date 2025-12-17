# Session Cookie Fix - Production Deployment

## Problem Diagnosed

**Symptom**: Users can login/signup but session not persisted. No cookies set in browser.

**Root Cause**: `.vercel.app` is a **Public Suffix** (like `.com` or `.co.uk`). Browsers **block** setting cookies for public suffixes as a security measure. When the auth server tried to set `Domain=.vercel.app`, the browser rejected it:

```
cookie '__Secure-better-auth.session_token' dropped,
domain 'robotook-auth.vercel.app' must not set cookies for 'vercel.app'
```

## Solution Applied

### Architecture Change: Same-Origin via Vercel Rewrites

Instead of cross-domain cookies, we use **Vercel rewrites** to make the auth server accessible on the same origin as the frontend.

```
Before (Cross-domain - FAILED):
  Frontend: https://robotook.vercel.app
  Auth:     https://robotook-auth.vercel.app  ❌ Different origin

After (Same-origin - WORKS):
  Frontend: https://robotook.vercel.app
  Auth:     https://robotook.vercel.app/api/auth/*  ✅ Same origin
            (rewrites to robotook-auth.vercel.app internally)
```

### Files Changed

#### 1. `website/vercel.json` (NEW)
```json
{
  "rewrites": [
    {
      "source": "/api/auth/:path*",
      "destination": "https://robotook-auth.vercel.app/api/auth/:path*"
    }
  ]
}
```

#### 2. `website/src/lib/auth-client.ts`
Changed from hardcoded auth server URL to same origin:
```typescript
// Before
const AUTH_URL = "https://robotook-auth.vercel.app";

// After
const AUTH_URL = typeof window !== "undefined"
  ? window.location.origin  // Uses same origin (robotook.vercel.app)
  : "https://robotook.vercel.app";
```

#### 3. `auth-server/src/auth.ts`
Removed domain setting, changed SameSite to "lax":
```typescript
advanced: {
  useSecureCookies: true,
  cookies: {
    session_token: {
      name: "better-auth.session_token",
      attributes: {
        sameSite: "lax",  // Changed from "none"
        secure: true,
        httpOnly: true,
        path: "/",
        // NO domain - stays on current origin only
      },
    },
  },
}
```

## Deployment Steps

### 1. Deploy Frontend with Rewrite Configuration
```bash
cd website
git add vercel.json src/lib/auth-client.ts
git commit -m "fix: Add Vercel rewrites for same-origin auth server"
git push origin master
vercel --prod
```

### 2. Deploy Auth Server with Updated Cookie Config
```bash
cd auth-server
git add src/auth.ts
git commit -m "fix: Remove cookie domain for same-origin deployment"
git push origin master
vercel --prod
```

### 3. Remove COOKIE_DOMAIN Environment Variable
In Vercel dashboard for **auth-server** project:
- Settings → Environment Variables
- **DELETE** `COOKIE_DOMAIN` if it exists
- Redeploy

## Testing

### 1. Test Signup
1. Visit `https://robotook.vercel.app/signup`
2. Create an account
3. **Check Browser DevTools**:
   - **Application > Cookies > https://robotook.vercel.app**
   - Should see: `better-auth.session_token` with:
     - `Domain`: `robotook.vercel.app` (NO `.vercel.app`)
     - `SameSite`: `Lax`
     - `Secure`: ✓
     - `HttpOnly`: ✓

### 2. Test Session Persistence
1. Stay logged in after signup
2. Refresh page → Should remain logged in ✅
3. Close browser and reopen → Should remain logged in ✅

### 3. Test Network Requests
Open DevTools Network tab:
- `/api/auth/sign-up/email` → Should return 200
- `/api/auth/get-session` → Should return 200 with user data
- Both requests should include `Cookie: better-auth.session_token=...`

## How It Works

```
┌─────────────────────────────────────────┐
│  User Browser                           │
│  Domain: robotook.vercel.app            │
│                                         │
│  Cookie: better-auth.session_token      │
│  Domain: robotook.vercel.app            │
│  SameSite: Lax, Secure, HttpOnly        │
└────────┬───────────────────────┬────────┘
         │                       │
         │ GET /signup           │ POST /api/auth/sign-up/email
         │                       │
┌────────▼───────────────────────▼────────┐
│  Vercel Edge (robotook.vercel.app)      │
│                                         │
│  Rewrite: /api/auth/* →                 │
│    https://robotook-auth.vercel.app/... │
└────────┬───────────────────────┬────────┘
         │                       │
         │ Static Site           │ Proxied to auth-server
         │                       │
┌────────▼────────────┐  ┌──────▼─────────────────┐
│  Docusaurus         │  │  Auth Server           │
│  (Static Files)     │  │  (Express + Better     │
│                     │  │   Auth + Neon DB)      │
└─────────────────────┘  └────────────────────────┘
```

## Why This Works

1. **Same Origin**: Browser sees all requests as `robotook.vercel.app`
2. **No Public Suffix**: Cookie domain is specific subdomain, not public suffix
3. **SameSite=Lax**: Works for same-origin requests
4. **Transparent Proxy**: Vercel rewrites handle the routing internally

## Rollback Plan

If issues occur, revert by:
1. Remove `website/vercel.json`
2. Set `AUTH_URL = "https://robotook-auth.vercel.app"` in auth-client.ts
3. Set `COOKIE_DOMAIN=` (empty) in auth-server env vars
4. Redeploy both services

## Sources & References

- [Better Auth Cookies Documentation](https://www.better-auth.com/docs/concepts/cookies)
- [Better Auth Client Configuration](https://www.better-auth.com/docs/concepts/client)
- [Public Suffix List](https://publicsuffix.org/)
- [Vercel Rewrites Documentation](https://vercel.com/docs/projects/project-configuration#rewrites)
