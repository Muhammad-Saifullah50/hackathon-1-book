# Data Model: Neon Database Migration with Better Auth

**Date**: 2025-12-13
**Feature**: 010-neon-database-migration

## Entity Relationship Diagram

```
┌─────────────────┐       ┌─────────────────┐       ┌─────────────────┐
│      User       │       │     Account     │       │     Session     │
├─────────────────┤       ├─────────────────┤       ├─────────────────┤
│ id (PK, UUID)   │◄──┬───│ userId (FK)     │       │ id (PK, UUID)   │
│ email           │   │   │ id (PK, UUID)   │       │ userId (FK)     │──►
│ emailVerified   │   │   │ providerId      │       │ token           │
│ name            │   │   │ accountId       │       │ expiresAt       │
│ image           │   │   │ password        │       │ ipAddress*      │
│ createdAt       │   │   │ createdAt       │       │ userAgent*      │
│ updatedAt       │   │   │ updatedAt       │       │ createdAt       │
└─────────────────┘   │   └─────────────────┘       │ updatedAt       │
        │             │                             └─────────────────┘
        │             │
        │ 1:1         │ 1:N                    * Session-specific security fields
        ▼             │
┌─────────────────┐   │
│   UserProfile   │   │
├─────────────────┤   │
│ id (PK, UUID)   │   │
│ user_id (FK)    │───┘
│ age_range       │
│ education_level │
│ tech_background │
│ primary_goal    │
│ learning_mode   │
│ learning_speed  │
│ time_per_week   │
│ preferred_lang  │
│ created_at      │
│ updated_at      │
└─────────────────┘
```

## Entities

### 1. User (Better Auth Core)

**Purpose**: Stores authenticated user identity information.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK, NOT NULL | Unique identifier |
| `email` | VARCHAR(255) | UNIQUE, NOT NULL | User email address |
| `emailVerified` | BOOLEAN | DEFAULT FALSE | Email verification status |
| `name` | VARCHAR(255) | NULLABLE | Display name |
| `image` | TEXT | NULLABLE | Profile image URL |
| `createdAt` | TIMESTAMP | NOT NULL, DEFAULT NOW() | Account creation time |
| `updatedAt` | TIMESTAMP | NOT NULL, DEFAULT NOW() | Last update time |

**Indexes**:
- Primary: `id`
- Unique: `email`

**Validation Rules**:
- Email must be valid format
- Email must be unique (case-insensitive)

---

### 2. Account (Better Auth Core)

**Purpose**: Links authentication providers to users (supports multiple auth methods per user).

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK, NOT NULL | Unique identifier |
| `userId` | UUID | FK(User.id), NOT NULL | Reference to user |
| `providerId` | VARCHAR(50) | NOT NULL | Auth provider (e.g., "credential", "google") |
| `accountId` | VARCHAR(255) | NOT NULL | Provider-specific account ID |
| `password` | TEXT | NULLABLE | Hashed password (for credential provider only) |
| `accessToken` | TEXT | NULLABLE | OAuth access token (for social providers) |
| `refreshToken` | TEXT | NULLABLE | OAuth refresh token (for social providers) |
| `expiresAt` | TIMESTAMP | NULLABLE | Token expiration (for social providers) |
| `createdAt` | TIMESTAMP | NOT NULL, DEFAULT NOW() | Record creation time |
| `updatedAt` | TIMESTAMP | NOT NULL, DEFAULT NOW() | Last update time |

**Indexes**:
- Primary: `id`
- Unique: (`providerId`, `accountId`)
- Index: `userId`

**Validation Rules**:
- Password required when providerId = "credential"
- accountId + providerId combination must be unique

---

### 3. Session (Better Auth Core)

**Purpose**: Tracks active user sessions for authentication and security auditing.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK, NOT NULL | Unique identifier |
| `userId` | UUID | FK(User.id), NOT NULL | Reference to user |
| `token` | TEXT | NOT NULL | Session token (used for refresh) |
| `expiresAt` | TIMESTAMP | NOT NULL | Session expiration (7 days from creation) |
| `ipAddress` | VARCHAR(45) | NULLABLE | **Security**: Client IP at login (for audit/anomaly detection) |
| `userAgent` | TEXT | NULLABLE | **Security**: Browser/device info (for session management UI) |
| `createdAt` | TIMESTAMP | NOT NULL, DEFAULT NOW() | Session creation time |
| `updatedAt` | TIMESTAMP | NOT NULL, DEFAULT NOW() | Last activity time |

**Why ipAddress and userAgent in Session (not User)?**
- These are **per-session** metadata, not user attributes
- Same user can have multiple sessions from different devices/locations
- Used for:
  - Security: "Login from new location" alerts
  - UX: "Manage active sessions" feature showing "Chrome on Windows, San Francisco"
  - Audit: Investigating suspicious access

**Indexes**:
- Primary: `id`
- Index: `userId`
- Index: `token`
- Index: `expiresAt` (for cleanup queries)

**State Transitions**:
```
[Created] → [Active] → [Expired/Invalidated]
                ↑
                └── [Refreshed]
```

---

### 4. UserProfile (Application-Specific)

**Purpose**: Stores learner profile data for personalized learning experience.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK, NOT NULL | Unique identifier |
| `user_id` | UUID | FK(User.id), UNIQUE, NOT NULL | 1:1 link to User |
| `age_range` | VARCHAR(20) | NULLABLE | Enum: under_18, 18_24, 25_34, 35_plus |
| `education_level` | VARCHAR(20) | NULLABLE | Enum: high_school, undergrad, masters, phd, self_taught |
| `tech_background` | VARCHAR(30) | NULLABLE | Enum: software_engineer, hardware_engineer, student, hobbyist |
| `primary_goal` | VARCHAR(30) | NULLABLE | Enum: career_switch, academic, hobby_project, startup_founder |
| `learning_mode` | VARCHAR(20) | NULLABLE | Enum: visual, textual, code_first |
| `learning_speed` | VARCHAR(20) | NULLABLE | Enum: intensive, balanced, casual |
| `time_per_week` | INTEGER | NULLABLE, >= 0 | Hours available per week |
| `preferred_language` | VARCHAR(10) | NOT NULL, DEFAULT 'en' | Language code |
| `created_at` | TIMESTAMP | NOT NULL, DEFAULT NOW() | Profile creation time |
| `updated_at` | TIMESTAMP | NOT NULL, DEFAULT NOW() | Last update time |

**Indexes**:
- Primary: `id`
- Unique: `user_id`

**Validation Rules**:
- user_id must reference existing User
- time_per_week must be non-negative
- Enum fields must match defined values

---

## Database Schema (SQL)

```sql
-- Better Auth tables (generated by CLI, shown for reference)

CREATE TABLE "user" (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) NOT NULL UNIQUE,
    "emailVerified" BOOLEAN DEFAULT FALSE,
    name VARCHAR(255),
    image TEXT,
    "createdAt" TIMESTAMP NOT NULL DEFAULT NOW(),
    "updatedAt" TIMESTAMP NOT NULL DEFAULT NOW()
);

CREATE TABLE account (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    "userId" UUID NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
    "providerId" VARCHAR(50) NOT NULL,
    "accountId" VARCHAR(255) NOT NULL,
    password TEXT,
    "accessToken" TEXT,
    "refreshToken" TEXT,
    "expiresAt" TIMESTAMP,
    "createdAt" TIMESTAMP NOT NULL DEFAULT NOW(),
    "updatedAt" TIMESTAMP NOT NULL DEFAULT NOW(),
    UNIQUE("providerId", "accountId")
);

CREATE TABLE session (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    "userId" UUID NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
    token TEXT NOT NULL,
    "expiresAt" TIMESTAMP NOT NULL,
    "ipAddress" VARCHAR(45),  -- Stores IPv4 or IPv6
    "userAgent" TEXT,          -- Browser/device info
    "createdAt" TIMESTAMP NOT NULL DEFAULT NOW(),
    "updatedAt" TIMESTAMP NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_account_user_id ON account("userId");
CREATE INDEX idx_session_user_id ON session("userId");
CREATE INDEX idx_session_token ON session(token);
CREATE INDEX idx_session_expires ON session("expiresAt");

-- Application table (existing, updated FK)

CREATE TABLE user_profile (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL UNIQUE REFERENCES "user"(id) ON DELETE CASCADE,
    age_range VARCHAR(20),
    education_level VARCHAR(20),
    tech_background VARCHAR(30),
    primary_goal VARCHAR(30),
    learning_mode VARCHAR(20),
    learning_speed VARCHAR(20),
    time_per_week INTEGER CHECK (time_per_week >= 0),
    preferred_language VARCHAR(10) NOT NULL DEFAULT 'en',
    created_at TIMESTAMP NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_user_profile_user_id ON user_profile(user_id);
```

---

## Migration Notes

### From Supabase to Neon

1. **User Migration**: Map Supabase `auth.users.id` → Better Auth `user.id`
2. **Profile Migration**: Update `user_profile.user_id` to reference new `user.id`
3. **Session Invalidation**: All existing Supabase sessions will be invalidated; users must re-login

### Data Integrity Constraints

- `ON DELETE CASCADE` ensures cleanup when user is deleted
- Unique constraints prevent duplicate accounts per provider
- Foreign keys maintain referential integrity
