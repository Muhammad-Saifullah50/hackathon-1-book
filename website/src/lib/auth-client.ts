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
const AUTH_URL =
  typeof window !== "undefined"
    ? (window as any).__AUTH_URL__
    : process.env.AUTH_URL;

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
