# Auth Server

Better Auth server for the Physical AI & Humanoid Robotics Learning Platform.

## Overview

This Express server hosts Better Auth API routes for user authentication:
- Sign up with email/password
- Sign in with email/password
- Sign out
- Session management
- JWT token issuance via JWKS

## Architecture

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   Docusaurus    │     │   auth-server   │     │    FastAPI      │
│  (Frontend)     │ ──► │   (This app)    │     │   (Backend)     │
│                 │     │   JWT issuer    │     │  JWT validator  │
└─────────────────┘     └────────┬────────┘     └────────┬────────┘
                                 │                       │
                                 └───────────┬───────────┘
                                             ▼
                                    ┌─────────────────┐
                                    │      Neon       │
                                    │   PostgreSQL    │
                                    └─────────────────┘
```

## Setup

### Prerequisites

- Node.js >= 20.0
- npm or pnpm
- Neon PostgreSQL database

### Installation

```bash
cd auth-server
npm install
```

### Environment Variables

Copy `.env.example` to `.env` and configure:

```bash
# Server Configuration
PORT=3001
NODE_ENV=development

# Neon Database Connection
DATABASE_URL=postgresql://username:password@host.neon.tech/dbname?sslmode=require

# Better Auth Configuration
BETTER_AUTH_SECRET=your-secret-key-at-least-32-characters-long
BETTER_AUTH_URL=http://localhost:3001

# CORS Origins (comma-separated)
CORS_ORIGINS=http://localhost:3000,http://localhost:8000

# JWT Configuration
JWT_EXPIRES_IN=7d
```

### Database Migration

Generate and apply Better Auth schema:

```bash
# Generate schema SQL
npm run generate

# Apply migration to Neon
npm run migrate
```

Then apply the user_profile migration:

```bash
psql $DATABASE_URL -f ../scripts/migrations/001_create_user_profile.sql
```

## Development

```bash
# Start development server with hot reload
npm run dev
```

Server runs at `http://localhost:3001`

## API Endpoints

All Better Auth routes are available at `/api/auth/*`:

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/auth/sign-up/email` | POST | Register with email/password |
| `/api/auth/sign-in/email` | POST | Login with email/password |
| `/api/auth/sign-out` | POST | Logout and invalidate session |
| `/api/auth/session` | GET | Get current session |
| `/api/auth/token` | POST | Get JWT token |
| `/api/auth/jwks` | GET | Get JSON Web Key Set |

### Health Check

```bash
curl http://localhost:3001/health
```

## Production Build

```bash
# Build TypeScript
npm run build

# Start production server
npm start
```

## Deployment

The auth-server should be deployed separately from:
- Docusaurus (static site)
- FastAPI backend

Recommended platforms:
- Railway
- Render
- Fly.io
- AWS ECS/Fargate

### Environment for Production

```bash
NODE_ENV=production
PORT=3001
DATABASE_URL=<neon-production-url>
BETTER_AUTH_SECRET=<strong-production-secret>
BETTER_AUTH_URL=https://auth.yourdomain.com
CORS_ORIGINS=https://yourdomain.com,https://api.yourdomain.com
```

## Security Notes

- `BETTER_AUTH_SECRET` must be at least 32 characters
- Use HTTPS in production
- Configure CORS strictly for production origins
- JWT tokens expire after 7 days (configurable)
- Sessions are stored in Neon database
