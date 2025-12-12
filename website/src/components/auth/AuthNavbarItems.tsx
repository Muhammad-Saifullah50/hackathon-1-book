// website/src/components/auth/AuthNavbarItems.tsx
import React from 'react';
import Link from '@docusaurus/Link';
import { Sun, Moon } from 'lucide-react';
import { useAuth } from '../../hooks/useAuth';
import { useSafeColorMode } from '../../hooks/useSafeColorMode';

const AuthNavbarItems: React.FC = () => {
  const { user, loading, logout } = useAuth();
  const { isDark, setColorMode } = useSafeColorMode();

  const toggleTheme = () => {
    setColorMode(isDark ? 'light' : 'dark');
  };

  // Theme toggle button (always shown) - square shape, flat style, no border
  const ThemeToggle = () => (
    <button
      onClick={toggleTheme}
      className={`flex items-center justify-center w-9 h-9 rounded-md transition-colors border-0 outline-none ${
        isDark
          ? 'bg-slate-800/60 hover:bg-slate-700/80'
          : 'bg-slate-100 hover:bg-slate-200'
      }`}
      aria-label={`Switch to ${isDark ? 'light' : 'dark'} mode`}
    >
      {isDark ? (
        <Sun className="w-4 h-4 text-amber-400" />
      ) : (
        <Moon className="w-4 h-4 text-slate-600" />
      )}
    </button>
  );

  if (loading) {
    return (
      <div className="flex items-center gap-3">
        <ThemeToggle />
        <span className={`text-sm ${isDark ? 'text-slate-400' : 'text-slate-500'}`}>
          Loading...
        </span>
      </div>
    );
  }

  if (user) {
    return (
      <div className="flex items-center gap-3">
        {/* Theme toggle - left */}
        <ThemeToggle />
        {/* User email - middle */}
        <span
          className={`text-sm px-3 py-1.5 rounded-lg ${
            isDark
              ? 'text-slate-300 bg-slate-800/40'
              : 'text-slate-600 bg-slate-100'
          }`}
        >
          {user.email}
        </span>
        {/* Logout - right - matches theme toggle style with destructive color */}
        <button
          onClick={() => logout()}
          className={`flex items-center justify-center w-9 h-9 rounded-md transition-colors border-0 outline-none ${
            isDark
              ? 'bg-slate-800/60 hover:bg-red-500/20 text-slate-400 hover:text-red-400'
              : 'bg-slate-100 hover:bg-red-50 text-slate-500 hover:text-red-600'
          }`}
          aria-label="Logout"
          title="Logout"
        >
          <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
            <path d="M9 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h4" />
            <polyline points="16 17 21 12 16 7" />
            <line x1="21" y1="12" x2="9" y2="12" />
          </svg>
        </button>
      </div>
    );
  }

  // Not logged in - show Theme Toggle, Login, Signup (left to right)
  return (
    <div className="flex items-center gap-3">
      {/* Theme toggle - left */}
      <ThemeToggle />
      {/* Login - middle */}
      <Link
        to="/login"
        className={`px-4 py-2 text-sm font-medium rounded-lg transition-colors ${
          isDark
            ? 'bg-slate-800/60 text-slate-300 hover:bg-slate-700/80 hover:text-emerald-400'
            : 'bg-slate-100 text-slate-600 hover:bg-slate-200 hover:text-emerald-600'
        }`}
      >
        Login
      </Link>
      {/* Sign Up - right */}
      <Link
        to="/signup"
        className={`px-4 py-2 text-sm font-medium rounded-lg text-white transition-colors ${
          isDark
            ? 'bg-emerald-500 hover:bg-emerald-400 shadow-md shadow-emerald-500/20'
            : 'bg-emerald-600 hover:bg-emerald-500 shadow-md shadow-emerald-600/20'
        }`}
      >
        Sign Up
      </Link>
    </div>
  );
};

export default AuthNavbarItems;
