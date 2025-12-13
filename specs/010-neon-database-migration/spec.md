# Feature Specification: Neon Database Migration with Better Auth

**Feature Branch**: `010-neon-database-migration`
**Created**: 2025-12-13
**Status**: Draft
**Input**: User description: "Migrate database to Neon and implement Better Auth for authentication with flow: Better Auth -> FastAPI -> Neon. Frontend is Docusaurus."

## Overview

This feature migrates the entire backend infrastructure from Supabase to a new stack:
- **Database**: Supabase PostgreSQL → Neon Serverless Postgres
- **Authentication**: Supabase Auth → Better Auth (TypeScript-based authentication framework)
- **Frontend**: Docusaurus (React-based static site generator)

The new architecture follows a clear separation:
- **Frontend (Docusaurus)**: Hosts Better Auth client for authentication UI, issues JWT tokens via swizzled Root component
- **Backend (FastAPI)**: Validates JWT tokens, handles business logic, interacts with Neon
- **Database (Neon)**: Stores both authentication tables (user, session, account) and application data (profiles, etc.)

### Authentication Flow

```
User → Docusaurus (Better Auth Client) → Better Auth API Routes
                                              ↓
                                         JWT Token
                                              ↓
                                      FastAPI Backend
                                              ↓
                                      Validates JWT via JWKS
                                              ↓
                                       Neon Database
```

### Technical Integration Notes

**Better Auth in Docusaurus:**
- Better Auth client is integrated via Docusaurus swizzling (`src/theme/Root.js`)
- Auth API routes can be served via a separate API server or serverless functions
- The `<Root>` component persists across navigation, ideal for auth state management
- Custom React pages in `/src/pages/` handle login/register UI

**JWT Flow:**
- Better Auth issues JWTs via `/api/auth/token` endpoint
- JWKS (JSON Web Key Set) available at `/api/auth/jwks`
- FastAPI validates tokens using `jose` library with remote JWKS

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Seamless Database Connection (Priority: P1)

As a system administrator, I want the application to connect to Neon database seamlessly so that existing functionality continues to work without disruption.

**Why this priority**: Core database connectivity is the foundation for all other features. Without successful database connection, no other functionality works.

**Independent Test**: Can be fully tested by verifying the backend successfully connects to Neon and executes a simple query, returning expected results.

**Acceptance Scenarios**:

1. **Given** the backend is configured with Neon connection credentials, **When** the application starts, **Then** a database connection is established successfully within 5 seconds
2. **Given** a valid Neon connection, **When** a health check endpoint is called, **Then** the system confirms database connectivity with a 200 status response
3. **Given** invalid Neon credentials, **When** the application attempts to connect, **Then** the system logs a clear error message and fails gracefully without crashing

---

### User Story 2 - User Registration with Better Auth (Priority: P1)

As a new user, I want to create an account using email and password so that I can access the personalized learning platform.

**Why this priority**: User registration is the entry point for the entire system. Without registration, no users can access the platform.

**Independent Test**: Can be fully tested by submitting registration form data and verifying account creation in Neon database.

**Acceptance Scenarios**:

1. **Given** a user with a valid email and password, **When** the user submits the registration form on Docusaurus, **Then** a new user account is created in Neon database via Better Auth
2. **Given** a user submits registration, **When** the account is created, **Then** the user, account, and session records are stored in Neon database
3. **Given** an email that already exists, **When** a user attempts to register, **Then** an appropriate error message is returned without exposing sensitive information

---

### User Story 3 - User Login and Session Management (Priority: P1)

As a registered user, I want to log in with my credentials so that I receive a session and can access protected resources.

**Why this priority**: Login is essential for user identity and accessing personalized features.

**Independent Test**: Can be fully tested by logging in with valid credentials and verifying session/token is returned.

**Acceptance Scenarios**:

1. **Given** a registered user with valid credentials, **When** the user logs in via Better Auth on Docusaurus, **Then** a valid session is created and stored in Neon
2. **Given** a successful login, **When** the session is created, **Then** a JWT token is available for API authentication
3. **Given** invalid credentials, **When** the user attempts to log in, **Then** authentication fails with an appropriate error message
4. **Given** an active session, **When** the user logs out, **Then** the session is invalidated in Neon database

---

### User Story 4 - JWT Token Validation in FastAPI (Priority: P1)

As a backend service, I want to validate JWT tokens from Better Auth so that I can authenticate API requests securely.

**Why this priority**: API authentication is critical for protecting user data and ensuring only authorized users can access resources.

**Independent Test**: Can be fully tested by sending requests with valid/invalid JWT tokens and verifying authentication responses.

**Acceptance Scenarios**:

1. **Given** a valid JWT token from Better Auth, **When** a request is made to a protected FastAPI endpoint, **Then** the request is authenticated and processed
2. **Given** an expired JWT token, **When** a request is made to a protected endpoint, **Then** a 401 Unauthorized response is returned
3. **Given** an invalid or tampered JWT token, **When** a request is made to a protected endpoint, **Then** a 401 Unauthorized response is returned
4. **Given** a request without a JWT token, **When** accessing a protected endpoint, **Then** a 401 Unauthorized response is returned

---

### User Story 5 - User Profile Operations (Priority: P1)

As a learner, I want to create, read, and update my profile so that my personalization settings are persisted in the new database.

**Why this priority**: Profile operations are critical for the personalized learning experience. Users must be able to save and retrieve their learning preferences.

**Independent Test**: Can be fully tested by creating a profile, retrieving it, updating it, and verifying all changes persist correctly.

**Acceptance Scenarios**:

1. **Given** an authenticated user with no existing profile, **When** the user submits profile creation data, **Then** a new profile is created and stored in Neon database
2. **Given** an authenticated user with an existing profile, **When** the user requests their profile, **Then** the correct profile data is returned from Neon
3. **Given** an authenticated user with an existing profile, **When** the user updates profile fields, **Then** the changes are persisted to Neon and subsequent reads return updated data

---

### User Story 6 - Data Migration From Supabase (Priority: P2)

As a system administrator, I want to migrate existing user and profile data from Supabase to Neon so that no user data is lost during the transition.

**Why this priority**: Data preservation is important but secondary to establishing the new infrastructure. Existing users need their data migrated.

**Independent Test**: Can be fully tested by exporting sample data from Supabase, importing to Neon, and verifying data integrity.

**Acceptance Scenarios**:

1. **Given** existing profiles in Supabase, **When** the migration script runs, **Then** all profiles are copied to Neon with matching data
2. **Given** migrated data in Neon, **When** comparing source and destination records, **Then** 100% of records match with no data corruption
3. **Given** a failed migration attempt, **When** reviewing logs, **Then** clear error messages identify which records failed and why

---

### Edge Cases

- What happens when Neon database is temporarily unavailable? System should return appropriate error responses and attempt reconnection
- How does the system handle concurrent profile updates? Database should use appropriate locking/conflict resolution
- What happens if connection pool is exhausted? System should queue requests or return a "service unavailable" response
- How does the system handle large data exports during migration? Migration should use batching to avoid memory issues
- What happens when Better Auth JWKS endpoint is unavailable? FastAPI should cache public keys and gracefully degrade
- How does the system handle JWT token refresh? Better Auth handles session refresh automatically; FastAPI validates new tokens seamlessly
- What happens when Docusaurus navigates between pages? Auth state persists via Root component across navigation

## Requirements *(mandatory)*

### Functional Requirements

#### Database Layer (Neon)
- **FR-001**: System MUST connect to Neon serverless Postgres using a secure connection string
- **FR-002**: System MUST support SSL/TLS encryption for all database connections
- **FR-003**: System MUST execute CRUD operations on all required tables (users, accounts, sessions, profiles)
- **FR-004**: System MUST handle database connection failures gracefully with appropriate error logging
- **FR-005**: System MUST provide a health check endpoint that verifies database connectivity
- **FR-006**: System MUST support connection pooling suitable for serverless environments

#### Authentication Layer (Better Auth + Docusaurus)
- **FR-007**: System MUST integrate Better Auth client in Docusaurus via swizzled Root component (`src/theme/Root.js`)
- **FR-008**: System MUST provide login and registration pages as custom React pages in Docusaurus (`/src/pages/`)
- **FR-009**: System MUST support email/password authentication via Better Auth
- **FR-010**: System MUST store authentication data (user, account, session tables) in Neon database
- **FR-011**: System MUST expose JWKS endpoint (`/api/auth/jwks`) for JWT public key distribution
- **FR-012**: System MUST issue JWT tokens for authenticated sessions via `/api/auth/token` endpoint
- **FR-013**: System MUST configure JWT with RS256 algorithm, 7-day expiration with session refresh, and appropriate issuer/audience
- **FR-014**: System MUST maintain auth state across Docusaurus page navigation

#### Backend Layer (FastAPI)
- **FR-015**: FastAPI MUST validate JWT tokens using Better Auth's JWKS endpoint
- **FR-016**: FastAPI MUST extract user identity from validated JWT tokens
- **FR-017**: FastAPI MUST protect all authenticated endpoints with JWT validation middleware
- **FR-018**: FastAPI MUST return 401 Unauthorized for invalid, expired, or missing tokens
- **FR-019**: FastAPI MUST cache JWKS public keys to reduce latency and handle JWKS endpoint unavailability

#### API Compatibility
- **FR-020**: System MUST maintain backward compatibility with existing API contracts (no changes to request/response formats for profile endpoints)
- **FR-021**: System MUST include a migration script to transfer data from Supabase to Neon

### Key Entities

- **User**: Better Auth user record with id (UUID), email, emailVerified, name, image, createdAt, updatedAt
- **Account**: Better Auth account linking with id, userId, provider, providerAccountId, etc.
- **Session**: Better Auth session with id, userId, token, expiresAt, ipAddress, userAgent
- **UserProfile**: Application-specific learner profile with user_id (UUID, foreign key to User.id, 1:1 relationship), display_name, learning_goals, experience_level, hardware_access, preferred_modules, created_at, updated_at

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All existing profile API endpoints continue to function with response times within 20% of current performance
- **SC-002**: Database connection is established on application startup within 5 seconds
- **SC-003**: Zero data loss during migration - 100% of source records exist in destination
- **SC-004**: Application handles 50 concurrent database operations without errors
- **SC-005**: All existing profile-related tests pass after migration without modification to test assertions
- **SC-006**: Database operations complete successfully with the first attempt 99% of the time under normal conditions
- **SC-007**: User can complete registration flow in under 5 seconds
- **SC-008**: User can complete login flow in under 3 seconds
- **SC-009**: JWT token validation completes in under 100ms
- **SC-010**: 99.9% of valid JWT tokens are accepted on first validation attempt
- **SC-011**: Auth state persists correctly across 100% of Docusaurus page navigations

## Assumptions

- Neon account and project have been created with appropriate credentials available
- Environment variables will be used for storing database connection strings and secrets (no hardcoded credentials)
- The backend uses Python with FastAPI framework
- The frontend uses Docusaurus (React-based static site generator)
- Better Auth API routes will be co-located with Docusaurus frontend (Node.js/serverless functions alongside static site)
- Better Auth will be configured with JWT plugin for token-based API authentication
- The existing profiles table schema will remain unchanged (only authentication tables are new)
- Better Auth's core schema (user, account, session, verification tables) will be created in Neon
- Docusaurus will be customized via swizzling to integrate Better Auth client

## Out of Scope

- Social OAuth providers (Google, GitHub, etc.) - can be added later via Better Auth plugins
- Two-factor authentication (2FA) - can be added later via Better Auth plugins
- Email verification flow - simplified for initial implementation
- Schema changes or data model modifications to existing profile data
- Real-time/subscription features (Supabase Realtime)
- Storage/file uploads (Supabase Storage)
- Major Docusaurus theme changes beyond auth integration

## Clarifications

### Session 2025-12-13

- Q: Where will Better Auth API routes be hosted? → A: Co-located with Docusaurus frontend (Better Auth runs alongside the React frontend, FastAPI only validates JWTs)
- Q: How are User and UserProfile related? → A: UserProfile.user_id is a foreign key to Better Auth User.id (1:1 relationship)
- Q: What is the JWT token expiration duration? → A: Medium-lived (7 days) with session refresh

## Dependencies

- Neon serverless Postgres account with project created
- Neon connection string with SSL enabled
- Python database driver compatible with Neon (psycopg2-binary or asyncpg)
- Better Auth npm package (`better-auth`) and JWT plugin
- Better Auth client package (`better-auth/client`)
- `jose` or `python-jose` library for JWT validation in FastAPI
- Docusaurus v3.x with swizzling capability
- Environment variable management for secure credential storage
- CORS configuration for Docusaurus frontend to access FastAPI backend
- Node.js runtime for Better Auth API routes (via Docusaurus SSR, serverless functions, or companion server)
