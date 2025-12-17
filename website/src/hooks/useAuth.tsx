// website/src/hooks/useAuth.tsx
/**
 * Authentication hook using Better Auth.
 *
 * This hook provides:
 * - User authentication state
 * - Login/signup/logout methods
 * - JWT token for API calls
 *
 * The auth flow:
 * 1. User authenticates via Better Auth (auth-server)
 * 2. Better Auth issues JWT token
 * 3. Frontend stores session and provides token for FastAPI calls
 */

import { useState, useEffect, useContext, createContext, useCallback } from 'react';
import { User, LoginCredentials, SignupCredentials } from '../types/auth';
import { authClient } from '../lib/auth-client';

interface AuthContextType {
  user: User | null;
  token: string | null;
  loading: boolean;
  error: string | null;
  login: (credentials: LoginCredentials) => Promise<string | null>;
  signup: (credentials: SignupCredentials) => Promise<string | null>;
  logout: () => Promise<void>;
  refreshSession: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider: React.FC<React.PropsWithChildren<{}>> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [token, setToken] = useState<string | null>(null);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);

  // Refresh session from Better Auth
  const refreshSession = useCallback(async () => {
    setLoading(true);
    try {
      const sessionResult = await authClient.getSession();

      if (sessionResult.data?.user) {
        const sessionUser = sessionResult.data.user;
        setUser({
          id: sessionUser.id,
          email: sessionUser.email,
        });

        // Get JWT token for API calls using Better Auth's token() method
        try {
          const tokenResult = await authClient.token();
          if (tokenResult.data?.token) {
            setToken(tokenResult.data.token);
            localStorage.setItem('access_token', tokenResult.data.token);
          }
        } catch (tokenErr) {
          // Session exists but no JWT - still authenticated
        }
      } else {
        setUser(null);
        setToken(null);
        localStorage.removeItem('access_token');
      }
    } catch (err) {
      console.error('Failed to refresh session:', err);
      setUser(null);
      setToken(null);
      localStorage.removeItem('access_token');
    } finally {
      setLoading(false);
    }
  }, []);

  // Initialize session on mount
  useEffect(() => {
    refreshSession();
  }, [refreshSession]);

  const login = async (credentials: LoginCredentials): Promise<string | null> => {
    setLoading(true);
    setError(null);
    try {
      const result = await authClient.signIn.email({
        email: credentials.email,
        password: credentials.password,
      });

      if (result.error) {
        const errorMessage = result.error.message || 'Login failed.';
        setError(errorMessage);
        return errorMessage;
      }

      // Refresh session to get user and token
      await refreshSession();
      return null;
    } catch (err) {
      const errorMessage = 'Network error or server unreachable.';
      setError(errorMessage);
      return errorMessage;
    } finally {
      setLoading(false);
    }
  };

  const signup = async (credentials: SignupCredentials): Promise<string | null> => {
    setLoading(true);
    setError(null);
    try {
      const result = await authClient.signUp.email({
        email: credentials.email,
        password: credentials.password,
        name: credentials.email.split('@')[0], // Default name from email
      });

      if (result.error) {
        // Handle duplicate email error
        if (result.error.message?.toLowerCase().includes('already exists')) {
          const errorMessage = 'An account with this email already exists.';
          setError(errorMessage);
          return errorMessage;
        }
        const errorMessage = result.error.message || 'Signup failed.';
        setError(errorMessage);
        return errorMessage;
      }

      // Refresh session to get user and token
      await refreshSession();
      return null;
    } catch (err) {
      const errorMessage = 'Network error or server unreachable.';
      setError(errorMessage);
      return errorMessage;
    } finally {
      setLoading(false);
    }
  };

  const logout = async () => {
    setLoading(true);
    setError(null);
    try {
      await authClient.signOut();
    } catch (err) {
      console.error('Logout error:', err);
      // Continue with local cleanup even if server call fails
    } finally {
      setUser(null);
      setToken(null);
      localStorage.removeItem('access_token');
      setLoading(false);
    }
  };

  return (
    <AuthContext.Provider value={{ user, token, loading, error, login, signup, logout, refreshSession }}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};
