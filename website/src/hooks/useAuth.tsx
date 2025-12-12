// website/src/hooks/useAuth.tsx
import { useState, useEffect, useContext, createContext } from 'react';
import { User, LoginCredentials, SignupCredentials } from '../types/auth';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext'; // Import Docusaurus context

interface AuthContextType {
  user: User | null;
  token: string | null; // Expose token
  loading: boolean;
  error: string | null;
  login: (credentials: LoginCredentials) => Promise<string | null>;
  signup: (credentials: SignupCredentials) => Promise<string | null>;
  logout: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider: React.FC<React.PropsWithChildren<{}>> = ({ children }) => {
  const { siteConfig } = useDocusaurusContext();
  const API_BASE_URL = (siteConfig.customFields?.backendUrl as string) || 'http://localhost:8000';

  const [user, setUser] = useState<User | null>(null);
  const [token, setToken] = useState<string | null>(null);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);

  // Initialize from localStorage
  useEffect(() => {
    const storedToken = localStorage.getItem('access_token');
    if (storedToken && storedToken !== "null" && storedToken !== "undefined") {
      setToken(storedToken);
      fetchUser(storedToken);
    } else {
      setLoading(false);
    }
  }, []);

  const fetchUser = async (accessToken: string) => {
    setLoading(true);
    try {
      const response = await fetch(`${API_BASE_URL}/auth/me`, {
        headers: {
          'Authorization': `Bearer ${accessToken}`
        }
      });
      if (response.ok) {
        const userData: User = await response.json();
        setUser(userData);
      } else {
        // Token invalid or expired
        localStorage.removeItem('access_token');
        setToken(null);
        setUser(null);
      }
    } catch (err) {
      setError('Failed to fetch user session.');
      // Keep token if it's just a network error? Maybe.
    } finally {
      setLoading(false);
    }
  };

  const login = async (credentials: LoginCredentials): Promise<string | null> => {
    setLoading(true);
    setError(null);
    try {
      const response = await fetch(`${API_BASE_URL}/auth/login`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(credentials),
      });
      if (response.ok) {
        const data = await response.json();
        const userData: User = data.user;
        const accessToken: string | null = data.access_token;
        
        setUser(userData);
        if (accessToken) {
            setToken(accessToken);
            localStorage.setItem('access_token', accessToken);
        } else {
            // Login successful but no token? (Shouldn't happen for login usually)
            setToken(null);
            localStorage.removeItem('access_token');
        }
        return null;
      } else {
        const errorData = await response.json();
        const errorMessage = errorData.detail || 'Login failed.';
        setError(errorMessage);
        return errorMessage;
      }
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
      const response = await fetch(`${API_BASE_URL}/auth/signup`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(credentials),
      });
      if (response.ok) {
        const data = await response.json();
        const userData: User = data.user;
        const accessToken: string | null = data.access_token;

        setUser(userData);
        
        if (accessToken) {
            setToken(accessToken);
            localStorage.setItem('access_token', accessToken);
        } else {
            // Signup successful but no token (likely email verification required)
            setToken(null);
            localStorage.removeItem('access_token');
            // We return null (success) but user/token state reflects reality
        }
        return null;
      } else {
        const errorData = await response.json();
        const errorMessage = errorData.detail || 'Signup failed.';
        setError(errorMessage);
        return errorMessage;
      }
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
      if (token) {
        await fetch(`${API_BASE_URL}/auth/logout`, { 
            method: 'POST',
            headers: {
                'Authorization': `Bearer ${token}`
            }
        });
      }
    } catch (err) {
      // Ignore network errors on logout
    } finally {
      setUser(null);
      setToken(null);
      localStorage.removeItem('access_token');
      setLoading(false);
    }
  };

  return (
    <AuthContext.Provider value={{ user, token, loading, error, login, signup, logout }}>
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