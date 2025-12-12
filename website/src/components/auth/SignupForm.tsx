// website/src/components/auth/SignupForm.tsx
import React, { useState } from 'react';
import { useAuth } from '../../hooks/useAuth';
import { useSafeColorMode } from '../../hooks/useSafeColorMode';
import { SignupCredentials } from '../../types/auth';
import { useHistory } from '@docusaurus/router';
import Link from '@docusaurus/Link';
import { UserPlus, Mail, Lock, AlertCircle, CheckCircle2 } from 'lucide-react';

const SignupForm: React.FC = () => {
  const { signup, loading, error: authError } = useAuth();
  const { isDark } = useSafeColorMode();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [localError, setLocalError] = useState<string | null>(null);
  const [isUserExists, setIsUserExists] = useState(false);
  const [successMessage, setSuccessMessage] = useState<string | null>(null);
  const history = useHistory();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLocalError(null);
    setSuccessMessage(null);
    setIsUserExists(false);

    const errorMessage = await signup({ email, password });

    if (errorMessage) {
      // Check if it's a "user exists" error
      if (errorMessage.toLowerCase().includes('already exists')) {
        setIsUserExists(true);
      }
      setLocalError(errorMessage);
    } else {
      // Signup successful - check if we have a token (auto-login) or need verification
      const storedToken = localStorage.getItem('access_token');
      if (storedToken) {
        history.push('/signup-wizard');
      } else {
        // User created but no token -> Email verification needed
        setSuccessMessage("Account created! Please check your email to verify your account.");
        setEmail('');
        setPassword('');
      }
    }
  };

  const displayError = localError || authError;

  return (
    <form onSubmit={handleSubmit} className="space-y-6">
      {/* Error message */}
      {displayError && (
        <div
          className={`flex flex-col gap-2 p-4 rounded-lg ${
            isDark
              ? 'bg-red-500/10 border border-red-500/20 text-red-400'
              : 'bg-red-50 border border-red-200 text-red-600'
          }`}
        >
          <div className="flex items-start gap-3">
            <AlertCircle className="w-5 h-5 flex-shrink-0 mt-0.5" />
            <p className="text-sm">{displayError}</p>
          </div>
          {isUserExists && (
            <Link
              to="/login"
              className={`text-sm ml-8 underline transition-colors ${
                isDark
                  ? 'text-emerald-400 hover:text-emerald-300'
                  : 'text-emerald-600 hover:text-emerald-500'
              }`}
            >
              Click here to login instead
            </Link>
          )}
        </div>
      )}

      {/* Success message */}
      {successMessage && (
        <div
          className={`flex items-start gap-3 p-4 rounded-lg ${
            isDark
              ? 'bg-emerald-500/10 border border-emerald-500/20 text-emerald-400'
              : 'bg-emerald-50 border border-emerald-200 text-emerald-600'
          }`}
        >
          <CheckCircle2 className="w-5 h-5 flex-shrink-0 mt-0.5" />
          <p className="text-sm">{successMessage}</p>
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
            placeholder="Create a strong password"
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
            Creating account...
          </>
        ) : (
          <>
            <UserPlus className="w-4 h-4" />
            Sign Up
          </>
        )}
      </button>
    </form>
  );
};

export default SignupForm;
