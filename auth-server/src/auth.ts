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

  // Advanced cookie settings for cross-domain production deployment
  advanced: {
    useSecureCookies: true, // Force secure cookies even in dev
    cookies: {
      session_token: {
        name: "better-auth.session_token",
        attributes: {
          sameSite: "none", // Required for cross-domain cookies
          secure: true, // Required for SameSite=None
          httpOnly: true,
          path: "/",
          domain: process.env.COOKIE_DOMAIN || undefined, // e.g., ".vercel.app" for shared cookies
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
