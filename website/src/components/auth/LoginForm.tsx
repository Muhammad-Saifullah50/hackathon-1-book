// website/src/components/auth/LoginForm.tsx
import React, { useState } from 'react';
import { useAuth } from '../../hooks/useAuth';
import { useSafeColorMode } from '../../hooks/useSafeColorMode';
import { LoginCredentials } from '../../types/auth';
import { useHistory } from '@docusaurus/router';
import { LogIn, Mail, Lock, AlertCircle } from 'lucide-react';

const BOOK_FIRST_PAGE = '/docs/module-01/overview';

const LoginForm: React.FC = () => {
  const { login, loading, error: authError } = useAuth();
  const { isDark } = useSafeColorMode();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [localError, setLocalError] = useState<string | null>(null);
  const history = useHistory();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLocalError(null);
    const errorMessage = await login({ email, password });
    if (errorMessage) {
      setLocalError(errorMessage);
    } else {
      // Login successful - redirect to the first page of the book
      history.push(BOOK_FIRST_PAGE);
    }
  };

  const displayError = localError || authError;

  return (
    <form onSubmit={handleSubmit} className="space-y-6">
      {/* Error message */}
      {displayError && (
        <div
          className={`flex items-start gap-3 p-4 rounded-lg ${
            isDark
              ? 'bg-red-500/10 border border-red-500/20 text-red-400'
              : 'bg-red-50 border border-red-200 text-red-600'
          }`}
        >
          <AlertCircle className="w-5 h-5 flex-shrink-0 mt-0.5" />
          <p className="text-sm">{displayError}</p>
        </div>
      )}

      {/* Email field */}
      <div className="space-y-2">
        <label
          htmlFor="email"
          className={`block text-sm font-medium ${
            isDark ? 'text-slate-300' : 'text-slate-700'
          }`}
        >
          Email
        </label>
        <div className="relative">
          <div
            className={`absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none ${
              isDark ? 'text-slate-500' : 'text-slate-400'
            }`}
          >
            <Mail className="w-5 h-5" />
          </div>
          <input
            type="email"
            id="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
            placeholder="you@example.com"
            className={`block w-full pl-10 pr-4 py-3 rounded-lg transition-colors ${
              isDark
                ? 'bg-slate-900 border-slate-700 text-slate-200 placeholder-slate-500 focus:border-emerald-500 focus:ring-1 focus:ring-emerald-500'
                : 'bg-white border-slate-300 text-slate-900 placeholder-slate-400 focus:border-emerald-500 focus:ring-1 focus:ring-emerald-500'
            } border outline-none`}
          />
        </div>
      </div>

      {/* Password field */}
      <div className="space-y-2">
        <label
          htmlFor="password"
          className={`block text-sm font-medium ${
            isDark ? 'text-slate-300' : 'text-slate-700'
          }`}
        >
          Password
        </label>
        <div className="relative">
          <div
            className={`absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none ${
              isDark ? 'text-slate-500' : 'text-slate-400'
            }`}
          >
            <Lock className="w-5 h-5" />
          </div>
          <input
            type="password"
            id="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
            placeholder="Enter your password"
            className={`block w-full pl-10 pr-4 py-3 rounded-lg transition-colors ${
              isDark
                ? 'bg-slate-900 border-slate-700 text-slate-200 placeholder-slate-500 focus:border-emerald-500 focus:ring-1 focus:ring-emerald-500'
                : 'bg-white border-slate-300 text-slate-900 placeholder-slate-400 focus:border-emerald-500 focus:ring-1 focus:ring-emerald-500'
            } border outline-none`}
          />
        </div>
      </div>

      {/* Submit button */}
      <button
        type="submit"
        disabled={loading}
        className={`w-full flex items-center justify-center gap-2 py-3 px-4 rounded-lg text-sm font-semibold transition-all duration-200 ${
          isDark
            ? 'bg-emerald-500 hover:bg-emerald-400 text-white shadow-lg shadow-emerald-500/20 disabled:bg-emerald-500/50'
            : 'bg-emerald-600 hover:bg-emerald-500 text-white shadow-lg shadow-emerald-600/20 disabled:bg-emerald-600/50'
        } disabled:cursor-not-allowed focus:outline-none focus:ring-2 focus:ring-emerald-500 focus:ring-offset-2 ${
          isDark ? 'focus:ring-offset-slate-900' : 'focus:ring-offset-white'
        }`}
      >
        {loading ? (
          <>
            <div className="w-4 h-4 border-2 border-white/30 border-t-white rounded-full animate-spin" />
            Logging in...
          </>
        ) : (
          <>
            <LogIn className="w-4 h-4" />
            Login
          </>
        )}
      </button>
    </form>
  );
};

export default LoginForm;
