/**
 * Authentication context for React components.
 *
 * Provides:
 * - Current user state
 * - Loading state
 * - Auth methods (login, logout, register)
 * - Token retrieval for API calls
 */

import React, {
  createContext,
  useContext,
  useState,
  useEffect,
  useCallback,
  ReactNode,
} from "react";
import { authClient, type Session, type User } from "../lib/auth-client";

interface AuthContextValue {
  /** Current authenticated user, null if not logged in */
  user: User | null;
  /** Current session, null if not logged in */
  session: Session | null;
  /** Whether auth state is being loaded */
  isLoading: boolean;
  /** Whether user is authenticated */
  isAuthenticated: boolean;
  /** Sign in with email and password */
  signIn: (email: string, password: string) => Promise<{ error?: string }>;
  /** Sign up with email, password, and name */
  signUp: (
    email: string,
    password: string,
    name: string
  ) => Promise<{ error?: string }>;
  /** Sign out current user */
  signOut: () => Promise<void>;
  /** Get JWT token for API calls */
  getToken: () => Promise<string | null>;
  /** Refresh session state */
  refreshSession: () => Promise<void>;
}

const AuthContext = createContext<AuthContextValue | null>(null);

interface AuthProviderProps {
  children: ReactNode;
}

/**
 * Authentication provider component.
 *
 * Wrap your app with this provider to enable authentication:
 *
 * ```tsx
 * <AuthProvider>
 *   <App />
 * </AuthProvider>
 * ```
 */
export function AuthProvider({ children }: AuthProviderProps) {
  const [user, setUser] = useState<User | null>(null);
  const [session, setSession] = useState<Session | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  // Load session on mount
  const refreshSession = useCallback(async () => {
    try {
      const result = await authClient.getSession();
      if (result.data) {
        setSession(result.data);
        setUser(result.data.user);
      } else {
        setSession(null);
        setUser(null);
      }
    } catch (error) {
      console.error("Failed to get session:", error);
      setSession(null);
      setUser(null);
    } finally {
      setIsLoading(false);
    }
  }, []);

  useEffect(() => {
    refreshSession();
  }, [refreshSession]);

  // Sign in handler
  const handleSignIn = useCallback(
    async (email: string, password: string): Promise<{ error?: string }> => {
      try {
        const result = await authClient.signIn.email({
          email,
          password,
        });

        if (result.error) {
          return { error: result.error.message || "Sign in failed" };
        }

        await refreshSession();
        return {};
      } catch (error) {
        console.error("Sign in error:", error);
        return { error: "An unexpected error occurred" };
      }
    },
    [refreshSession]
  );

  // Sign up handler
  const handleSignUp = useCallback(
    async (
      email: string,
      password: string,
      name: string
    ): Promise<{ error?: string }> => {
      try {
        const result = await authClient.signUp.email({
          email,
          password,
          name,
        });

        if (result.error) {
          // Handle duplicate email error
          if (result.error.message?.includes("already exists")) {
            return { error: "An account with this email already exists" };
          }
          return { error: result.error.message || "Sign up failed" };
        }

        await refreshSession();
        return {};
      } catch (error) {
        console.error("Sign up error:", error);
        return { error: "An unexpected error occurred" };
      }
    },
    [refreshSession]
  );

  // Sign out handler
  const handleSignOut = useCallback(async () => {
    try {
      await authClient.signOut();
      setUser(null);
      setSession(null);
    } catch (error) {
      console.error("Sign out error:", error);
      // Clear local state even if server call fails
      setUser(null);
      setSession(null);
    }
  }, []);

  // Get JWT token for API calls
  const getToken = useCallback(async (): Promise<string | null> => {
    try {
      // Better Auth's JWT plugin provides token via token() method
      const result = await authClient.token();
      return result.data?.token || null;
    } catch (error) {
      console.error("Failed to get token:", error);
      return null;
    }
  }, []);

  const value: AuthContextValue = {
    user,
    session,
    isLoading,
    isAuthenticated: !!user,
    signIn: handleSignIn,
    signUp: handleSignUp,
    signOut: handleSignOut,
    getToken,
    refreshSession,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

/**
 * Hook to access authentication context.
 *
 * Must be used within an AuthProvider.
 *
 * ```tsx
 * function MyComponent() {
 *   const { user, isAuthenticated, signOut } = useAuth();
 *
 *   if (!isAuthenticated) {
 *     return <LoginPrompt />;
 *   }
 *
 *   return <div>Hello, {user.name}!</div>;
 * }
 * ```
 */
export function useAuth(): AuthContextValue {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error("useAuth must be used within an AuthProvider");
  }
  return context;
}

export default AuthContext;
