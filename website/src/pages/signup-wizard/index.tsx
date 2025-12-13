// website/src/pages/signup-wizard/index.tsx
import React, { useState } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import { useAuth } from '../../hooks/useAuth';
import { useProfile } from '../../hooks/useProfile';
import { useSafeColorMode } from '../../hooks/useSafeColorMode';
import ProfileWizardStep1 from '../../components/profile/ProfileWizardStep1';
import ProfileWizardStep2 from '../../components/profile/ProfileWizardStep2';
import ProfileWizardStep3 from '../../components/profile/ProfileWizardStep3';
import { UserProfileUpdate } from '../../data/profile-schema';
import { UserCircle, LogIn, SkipForward, Shield, Loader2 } from 'lucide-react';

const BOOK_FIRST_PAGE = '/docs/module-01/overview';

const SignupWizardPage: React.FC = () => {
  const { user } = useAuth();
  const { createOrUpdateProfile, loadingProfile } = useProfile();
  const { isDark } = useSafeColorMode();
  const [currentStep, setCurrentStep] = useState(1);
  const [formData, setFormData] = useState<UserProfileUpdate>({});

  if (!user) {
    return (
      <Layout title="Signup Wizard">
        <div
          className={`min-h-[calc(100vh-60px)] ${
            isDark ? 'bg-slate-950' : 'bg-slate-50'
          }`}
        >
          <div className="container mx-auto px-4 py-16">
            <div className="flex justify-center">
              <div className="w-full max-w-lg text-center">
                <div
                  className={`rounded-2xl p-8 ${
                    isDark
                      ? 'bg-slate-900/60 border border-white/10'
                      : 'bg-white border border-slate-200 shadow-lg'
                  }`}
                >
                  <div
                    className={`inline-flex items-center justify-center w-16 h-16 rounded-xl mb-4 ${
                      isDark
                        ? 'bg-amber-500/10 border border-amber-500/20'
                        : 'bg-amber-100 border border-amber-200'
                    }`}
                  >
                    <LogIn
                      className={`w-8 h-8 ${
                        isDark ? 'text-amber-400' : 'text-amber-600'
                      }`}
                    />
                  </div>
                  <h1
                    className={`text-2xl font-bold tracking-tight mb-3 ${
                      isDark ? 'text-slate-50' : 'text-slate-900'
                    }`}
                  >
                    Authentication Required
                  </h1>
                  <p
                    className={`mb-6 ${
                      isDark ? 'text-slate-400' : 'text-slate-600'
                    }`}
                  >
                    Please log in or sign up to continue setting up your learner profile.
                  </p>
                  <div className="flex flex-col sm:flex-row gap-3 justify-center">
                    <Link
                      to="/login"
                      className={`inline-flex items-center justify-center gap-2 px-6 py-3 rounded-lg text-sm font-semibold transition-all ${
                        isDark
                          ? 'bg-slate-800 hover:bg-slate-700 text-slate-300'
                          : 'bg-slate-100 hover:bg-slate-200 text-slate-700'
                      }`}
                    >
                      <LogIn className="w-4 h-4" />
                      Log In
                    </Link>
                    <Link
                      to="/signup"
                      className={`inline-flex items-center justify-center gap-2 px-6 py-3 rounded-lg text-sm font-semibold transition-all ${
                        isDark
                          ? 'bg-emerald-500 hover:bg-emerald-400 text-white'
                          : 'bg-emerald-600 hover:bg-emerald-500 text-white'
                      }`}
                    >
                      Sign Up
                    </Link>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  const handleNext = (data: Partial<UserProfileUpdate>) => {
    setFormData((prev) => ({ ...prev, ...data }));
    setCurrentStep((prev) => prev + 1);
  };

  const handleBack = () => {
    setCurrentStep((prev) => prev - 1);
  };

  const handleSubmit = async (data: Partial<UserProfileUpdate>) => {
    const finalData = { ...formData, ...data };
    await createOrUpdateProfile(finalData);
    // Redirect to the first page of the book after successful profile setup
    window.location.href = BOOK_FIRST_PAGE;
  };

  const handleSkip = async () => {
    // Skip profile setup and go directly to the book
    window.location.href = BOOK_FIRST_PAGE;
  };

  return (
    <Layout title="Learner Profile Setup">
      <div
        className={`min-h-[calc(100vh-60px)] ${
          isDark ? 'bg-slate-950' : 'bg-slate-50'
        }`}
      >
        {/* Background decorative elements */}
        <div className="absolute inset-0 overflow-hidden pointer-events-none">
          <div
            className={`absolute top-1/4 left-1/3 w-96 h-96 rounded-full blur-3xl ${
              isDark ? 'bg-emerald-500/5' : 'bg-emerald-500/10'
            }`}
          />
          <div
            className={`absolute bottom-1/4 right-1/3 w-64 h-64 rounded-full blur-3xl ${
              isDark ? 'bg-amber-500/5' : 'bg-amber-500/10'
            }`}
          />
        </div>

        <div className="container mx-auto px-4 py-12 relative z-10">
          <div className="flex justify-center">
            <div className="w-full max-w-xl">
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
                    <UserCircle
                      className={`w-8 h-8 ${
                        isDark ? 'text-emerald-400' : 'text-emerald-600'
                      }`}
                    />
                  </div>
                  <h1
                    className={`text-2xl sm:text-3xl font-bold tracking-tight ${
                      isDark ? 'text-slate-50' : 'text-slate-900'
                    }`}
                  >
                    Setup Your Learner Profile
                  </h1>
                  <p
                    className={`mt-2 ${
                      isDark ? 'text-slate-400' : 'text-slate-600'
                    }`}
                  >
                    Personalize your learning experience
                  </p>
                </div>

                {/* Progress indicator */}
                <div className="mb-8">
                  <div className="flex items-center justify-between mb-2">
                    <span
                      className={`text-sm font-medium ${
                        isDark ? 'text-slate-400' : 'text-slate-600'
                      }`}
                    >
                      Step {currentStep} of 3
                    </span>
                    <span
                      className={`text-sm ${
                        isDark ? 'text-slate-500' : 'text-slate-500'
                      }`}
                    >
                      {Math.round((currentStep / 3) * 100)}% complete
                    </span>
                  </div>
                  <div
                    className={`h-2 rounded-full overflow-hidden ${
                      isDark ? 'bg-slate-800' : 'bg-slate-200'
                    }`}
                  >
                    <div
                      className={`h-full rounded-full transition-all duration-500 ${
                        isDark
                          ? 'bg-gradient-to-r from-emerald-500 to-emerald-400'
                          : 'bg-gradient-to-r from-emerald-600 to-emerald-500'
                      }`}
                      style={{ width: `${(currentStep / 3) * 100}%` }}
                    />
                  </div>
                </div>

                {/* Loading state */}
                {loadingProfile && (
                  <div className="flex flex-col items-center justify-center py-12">
                    <Loader2
                      className={`w-8 h-8 animate-spin ${
                        isDark ? 'text-emerald-400' : 'text-emerald-600'
                      }`}
                    />
                    <p
                      className={`mt-4 ${
                        isDark ? 'text-slate-400' : 'text-slate-600'
                      }`}
                    >
                      Loading your profile...
                    </p>
                  </div>
                )}

                {/* Wizard steps */}
                {!loadingProfile && (
                  <>
                    {currentStep === 1 && (
                      <ProfileWizardStep1 onNext={handleNext} initialData={formData} />
                    )}
                    {currentStep === 2 && (
                      <ProfileWizardStep2
                        onNext={handleNext}
                        onBack={handleBack}
                        initialData={formData}
                      />
                    )}
                    {currentStep === 3 && (
                      <ProfileWizardStep3
                        onSubmit={handleSubmit}
                        onBack={handleBack}
                        initialData={formData}
                      />
                    )}
                  </>
                )}

                {/* Skip button */}
                <div className="mt-8 text-center">
                  <button
                    onClick={handleSkip}
                    className={`inline-flex items-center gap-2 text-sm font-medium px-4 py-2.5 rounded-lg transition-all duration-200 ${
                      isDark
                        ? 'text-slate-200 hover:text-white bg-slate-800 hover:bg-slate-700 border border-slate-600 hover:border-slate-500'
                        : 'text-slate-700 hover:text-slate-900 bg-slate-100 hover:bg-slate-200 border border-slate-300 hover:border-slate-400'
                    }`}
                  >
                    <SkipForward className="w-4 h-4" />
                    <span>Skip for now</span>
                    <span className={`text-xs ${isDark ? 'text-amber-400' : 'text-amber-600'}`}>
                      (Features will be limited)
                    </span>
                  </button>
                </div>

                {/* Privacy notice */}
                <div
                  className={`mt-6 pt-6 border-t border-dashed ${
                    isDark ? 'border-slate-700' : 'border-slate-200'
                  }`}
                >
                  <div
                    className={`flex items-start gap-3 text-xs ${
                      isDark ? 'text-slate-500' : 'text-slate-500'
                    }`}
                  >
                    <Shield className="w-4 h-4 flex-shrink-0 mt-0.5" />
                    <p>
                      <strong>Privacy Notice:</strong> Your data is used ONLY for
                      tailoring the textbook experience. We never share your
                      information with third parties.
                    </p>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default SignupWizardPage;
