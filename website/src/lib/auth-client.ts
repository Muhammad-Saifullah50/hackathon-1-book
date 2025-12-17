/**
 * Better Auth client configuration for Docusaurus frontend.
 *
 * Provides authentication methods:
 * - signUp: Register new user
 * - signIn: Login with email/password
 * - signOut: Logout and invalidate session
 * - getSession: Get current session
 * - token: Get JWT token for API calls (via jwtClient plugin)
 */

import { createAuthClient } from "better-auth/react";
import { jwtClient } from "better-auth/client/plugins";

// Auth server URL
// Using relative path via Vercel rewrite to avoid cross-domain cookie issues
// Vercel rewrites /api/auth/* to https://robotook-auth.vercel.app/api/auth/*
const AUTH_URL = typeof window !== "undefined" ? window.location.origin : "https://robotook.vercel.app";

/**
 * Better Auth client instance with JWT plugin.
 *
 * Usage:
 *   import { authClient } from '@site/src/lib/auth-client';
 *
 *   // Sign up
 *   const { data, error } = await authClient.signUp.email({
 *     email: 'user@example.com',
 *     password: 'password123',
 *     name: 'User Name',
 *   });
 *
 *   // Sign in
 *   const { data, error } = await authClient.signIn.email({
 *     email: 'user@example.com',
 *     password: 'password123',
 *   });
 *
 *   // Sign out
 *   await authClient.signOut();
 *
 *   // Get session
 *   const session = await authClient.getSession();
 *
 *   // Get JWT token for API calls
 *   const { data } = await authClient.token();
 *   const jwtToken = data?.token;
 */
export const authClient = createAuthClient({
  baseURL: AUTH_URL,
  plugins: [jwtClient()],
  fetchOptions: {
    credentials: "include", // Required for cross-domain cookie sending
  },
});

// Export individual methods for convenience
export const {
  signUp,
  signIn,
  signOut,
  useSession,
  getSession,
} = authClient;

// Type exports
export type Session = typeof authClient.$Infer.Session;
export type User = typeof authClient.$Infer.Session.user;
