// website/src/components/profile/PersonalizationBar.tsx
import React from 'react';
import { useProfile } from '../../hooks/useProfile.tsx';
import { useSafeColorMode } from '../../hooks/useSafeColorMode';
import { Sparkles, Wand2 } from 'lucide-react';

const PersonalizationBar: React.FC = () => {
  const { profile, loadingProfile, profileError } = useProfile();
  const { isDark } = useSafeColorMode();

  // Debug logging - remove in production
  console.log('[PersonalizationBar] loadingProfile:', loadingProfile, 'profile:', profile, 'error:', profileError);

  if (loadingProfile) {
    return null; // Or a loading spinner
  }

  if (!profile) {
    // No profile - user either not logged in or hasn't completed wizard
    return null;
  }

  const handlePersonalizeClick = () => {
    // This is the trigger for future logic
    console.log('Personalize Page clicked for user:', profile.user_id);
    console.log('User preferences:', {
      time_per_week: profile.time_per_week,
      tech_background: profile.tech_background,
    });
    alert('Personalization logic triggered! (Implementation deferred)');
  };

  return (
    <div
      className={`flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4 p-4 rounded-xl mb-6 ${
        isDark
          ? 'bg-gradient-to-r from-emerald-500/10 to-amber-500/10 border border-emerald-500/20'
          : 'bg-gradient-to-r from-emerald-50 to-amber-50 border border-emerald-200'
      }`}
    >
      <div className="flex items-start gap-3">
        <div
          className={`flex items-center justify-center w-10 h-10 rounded-lg flex-shrink-0 ${
            isDark ? 'bg-emerald-500/20' : 'bg-emerald-100'
          }`}
        >
          <Sparkles
            className={`w-5 h-5 ${isDark ? 'text-emerald-400' : 'text-emerald-600'}`}
          />
        </div>
        <div>
          <p
            className={`font-semibold ${
              isDark ? 'text-slate-200' : 'text-slate-800'
            }`}
          >
            Personalize this page?
          </p>
          <p
            className={`text-sm ${isDark ? 'text-slate-400' : 'text-slate-600'}`}
          >
            We can tailor content based on your{' '}
            <span className={`font-medium ${isDark ? 'text-emerald-400' : 'text-emerald-600'}`}>
              {profile.tech_background
                ? profile.tech_background.replace('_', ' ')
                : 'profile'}
            </span>{' '}
            background.
          </p>
        </div>
      </div>
      <button
        onClick={handlePersonalizeClick}
        className={`flex items-center gap-2 px-4 py-2 rounded-lg text-sm font-semibold transition-all duration-200 ${
          isDark
            ? 'bg-emerald-500 hover:bg-emerald-400 text-white shadow-lg shadow-emerald-500/20'
            : 'bg-emerald-600 hover:bg-emerald-500 text-white shadow-lg shadow-emerald-600/20'
        } focus:outline-none focus:ring-2 focus:ring-emerald-500 focus:ring-offset-2 ${
          isDark ? 'focus:ring-offset-slate-900' : 'focus:ring-offset-white'
        }`}
      >
        <Wand2 className="w-4 h-4" />
        Personalize Page
      </button>
    </div>
  );
};

export default PersonalizationBar;
