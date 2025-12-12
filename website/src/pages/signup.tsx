// website/src/pages/signup.tsx
import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import SignupForm from '../components/auth/SignupForm';
import { useSafeColorMode } from '../hooks/useSafeColorMode';
import { UserPlus, LogIn } from 'lucide-react';

const SignupPage: React.FC = () => {
  const { isDark } = useSafeColorMode();

  return (
    <Layout title="Sign Up">
      <div
        className={`min-h-[calc(100vh-60px)] ${
          isDark ? 'bg-slate-950' : 'bg-slate-50'
        }`}
      >
        {/* Background decorative elements */}
        <div className="absolute inset-0 overflow-hidden pointer-events-none">
          <div
            className={`absolute top-1/3 right-1/4 w-96 h-96 rounded-full blur-3xl ${
              isDark ? 'bg-emerald-500/5' : 'bg-emerald-500/10'
            }`}
          />
          <div
            className={`absolute bottom-1/3 left-1/4 w-64 h-64 rounded-full blur-3xl ${
              isDark ? 'bg-amber-500/5' : 'bg-amber-500/10'
            }`}
          />
        </div>

        <div className="container mx-auto px-4 py-16 relative z-10">
          <div className="flex justify-center">
            <div className="w-full max-w-md">
              {/* Card container */}
              <div
                className={`rounded-2xl p-8 ${
                  isDark
                    ? 'bg-slate-900/60 border border-white/10 shadow-2xl shadow-emerald-500/5'
                    : 'bg-white border border-slate-200 shadow-xl'
                }`}
              >
                {/* Header */}
                <div className="text-center mb-8">
                  <div
                    className={`inline-flex items-center justify-center w-16 h-16 rounded-xl mb-4 ${
                      isDark
                        ? 'bg-emerald-500/10 border border-emerald-500/20'
                        : 'bg-emerald-100 border border-emerald-200'
                    }`}
                  >
                    <UserPlus
                      className={`w-8 h-8 ${
                        isDark ? 'text-emerald-400' : 'text-emerald-600'
                      }`}
                    />
                  </div>
                  <h1
                    className={`text-3xl font-bold tracking-tight ${
                      isDark ? 'text-slate-50' : 'text-slate-900'
                    }`}
                  >
                    Create Account
                  </h1>
                  <p
                    className={`mt-2 ${
                      isDark ? 'text-slate-400' : 'text-slate-600'
                    }`}
                  >
                    Start your journey into Physical AI
                  </p>
                </div>

                {/* Form */}
                <SignupForm />

                {/* Footer */}
                <div
                  className={`mt-8 pt-6 border-t border-dashed ${
                    isDark ? 'border-slate-700' : 'border-slate-200'
                  }`}
                >
                  <p
                    className={`text-center text-sm ${
                      isDark ? 'text-slate-400' : 'text-slate-600'
                    }`}
                  >
                    Already have an account?{' '}
                    <Link
                      to="/login"
                      className={`font-semibold inline-flex items-center gap-1 transition-colors ${
                        isDark
                          ? 'text-emerald-400 hover:text-emerald-300'
                          : 'text-emerald-600 hover:text-emerald-500'
                      }`}
                    >
                      <LogIn className="w-4 h-4" />
                      Log in
                    </Link>
                  </p>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default SignupPage;
