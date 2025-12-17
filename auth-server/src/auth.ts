import { betterAuth } from "better-auth";
import { jwt } from "better-auth/plugins";
import { pool } from "./db.js";

/**
 * Better Auth server configuration with Neon PostgreSQL
 *
 * Features:
 * - Email/password authentication
 * - JWT tokens with RS256 algorithm
 * - 7-day token expiration with session refresh
 * - JWKS endpoint for public key distribution
 */
export const auth = betterAuth({
  database: pool,

  // Email and password authentication (no email verification)
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
    maxPasswordLength: 128,
  },

  // Session configuration
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days in seconds
    updateAge: 60 * 60 * 24, // Update session every 24 hours
    cookieCache: {
      enabled: true,
      maxAge: 60 * 5, // 5 minutes cache
    },
  },

  // Advanced cookie settings for production deployment
  // NOTE: Cannot use shared domain (.vercel.app) as it's a public suffix
  // Auth must be on same origin as frontend (via Vercel rewrites)
  advanced: {
    useSecureCookies: true, // Force secure cookies
    cookies: {
      session_token: {
        name: "better-auth.session_token",
        attributes: {
          sameSite: "lax", // Lax for same-origin (changed from "none")
          secure: true,
          httpOnly: true,
          path: "/",
          // domain: undefined - No domain means cookie stays on current origin only
        },
      },
    },
  },

  // JWT plugin for API authentication
  plugins: [
    jwt({
      jwt: {
        expirationTime: "7d",
        issuer: process.env.BETTER_AUTH_URL || "http://localhost:3001",
        audience: process.env.BETTER_AUTH_URL || "http://localhost:3001",
      },
      jwks: {
        keyPairConfig: {
          alg: "RS256",
        },
      },
    }),
  ],

  // Trust proxy headers (for deployment behind reverse proxy)
  trustedOrigins: [
    "http://localhost:3000",
    "http://localhost:8000",
    "https://robotook.vercel.app",
    "https://robotook-auth.vercel.app",
    ...(process.env.CORS_ORIGINS?.split(",") || []),
  ].filter(Boolean),
});

export type Auth = typeof auth;
