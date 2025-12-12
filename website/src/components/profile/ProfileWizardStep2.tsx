// website/src/components/profile/ProfileWizardStep2.tsx
import React, { useState } from 'react';
import { UserProfileUpdate, EducationLevelSchema, TechBackgroundSchema } from '../../data/profile-schema';
import { useSafeColorMode } from '../../hooks/useSafeColorMode';
import { z } from 'zod';
import { ChevronRight, ChevronLeft, AlertCircle, GraduationCap, Cpu } from 'lucide-react';

interface Props {
  onNext: (data: Partial<UserProfileUpdate>) => void;
  onBack: () => void;
  initialData: Partial<UserProfileUpdate>;
}

const ProfileWizardStep2: React.FC<Props> = ({ onNext, onBack, initialData }) => {
  const { isDark } = useSafeColorMode();
  const [educationLevel, setEducationLevel] = useState(initialData.education_level || '');
  const [techBackground, setTechBackground] = useState(initialData.tech_background || '');
  const [error, setError] = useState<string | null>(null);

  const handleNext = (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    try {
      // Validate with Zod
      EducationLevelSchema.parse(educationLevel);
      TechBackgroundSchema.parse(techBackground);

      onNext({
        education_level: educationLevel as any,
        tech_background: techBackground as any,
      });
    } catch (e) {
      if (e instanceof z.ZodError) {
        setError(e.issues[0].message);
      } else {
        setError('An unexpected error occurred.');
      }
    }
  };

  return (
    <form onSubmit={handleNext} className="space-y-6">
      {/* Step header */}
      <div className="space-y-2">
        <div className="flex items-center gap-2">
          <div
            className={`flex items-center justify-center w-8 h-8 rounded-full text-sm font-bold ${
              isDark
                ? 'bg-emerald-500/20 text-emerald-400'
                : 'bg-emerald-100 text-emerald-600'
            }`}
          >
            2
          </div>
          <h2
            className={`text-xl font-semibold tracking-tight ${
              isDark ? 'text-slate-100' : 'text-slate-900'
            }`}
          >
            Your Background
          </h2>
        </div>
        <p className={`text-sm ${isDark ? 'text-slate-400' : 'text-slate-600'}`}>
          Tell us about your education and technical experience.
        </p>
      </div>

      {/* Error message */}
      {error && (
        <div
          className={`flex items-start gap-3 p-4 rounded-lg ${
            isDark
              ? 'bg-red-500/10 border border-red-500/20 text-red-400'
              : 'bg-red-50 border border-red-200 text-red-600'
          }`}
        >
          <AlertCircle className="w-5 h-5 flex-shrink-0 mt-0.5" />
          <p className="text-sm">{error}</p>
        </div>
      )}

      {/* Education Level field */}
      <div className="space-y-2">
        <label
          htmlFor="education_level"
          className={`block text-sm font-medium ${
            isDark ? 'text-slate-300' : 'text-slate-700'
          }`}
        >
          Education Level
        </label>
        <div className="relative">
          <div
            className={`absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none ${
              isDark ? 'text-slate-500' : 'text-slate-400'
            }`}
          >
            <GraduationCap className="w-5 h-5" />
          </div>
          <select
            id="education_level"
            value={educationLevel}
            onChange={(e) => setEducationLevel(e.target.value)}
            required
            className={`block w-full pl-10 pr-10 py-3 rounded-lg transition-colors appearance-none ${
              isDark
                ? 'bg-slate-900 border-slate-700 text-slate-200 focus:border-emerald-500 focus:ring-1 focus:ring-emerald-500'
                : 'bg-white border-slate-300 text-slate-900 focus:border-emerald-500 focus:ring-1 focus:ring-emerald-500'
            } border outline-none cursor-pointer`}
          >
            <option value="" disabled>
              Select Education Level
            </option>
            {EducationLevelSchema.options.map((option) => (
              <option key={option} value={option}>
                {option.replace('_', ' ')}
              </option>
            ))}
          </select>
          <div
            className={`absolute inset-y-0 right-0 pr-3 flex items-center pointer-events-none ${
              isDark ? 'text-slate-500' : 'text-slate-400'
            }`}
          >
            <ChevronRight className="w-5 h-5 rotate-90" />
          </div>
        </div>
      </div>

      {/* Tech Background field */}
      <div className="space-y-2">
        <label
          htmlFor="tech_background"
          className={`block text-sm font-medium ${
            isDark ? 'text-slate-300' : 'text-slate-700'
          }`}
        >
          Tech Background
        </label>
        <div className="relative">
          <div
            className={`absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none ${
              isDark ? 'text-slate-500' : 'text-slate-400'
            }`}
          >
            <Cpu className="w-5 h-5" />
          </div>
          <select
            id="tech_background"
            value={techBackground}
            onChange={(e) => setTechBackground(e.target.value)}
            required
            className={`block w-full pl-10 pr-10 py-3 rounded-lg transition-colors appearance-none ${
              isDark
                ? 'bg-slate-900 border-slate-700 text-slate-200 focus:border-emerald-500 focus:ring-1 focus:ring-emerald-500'
                : 'bg-white border-slate-300 text-slate-900 focus:border-emerald-500 focus:ring-1 focus:ring-emerald-500'
            } border outline-none cursor-pointer`}
          >
            <option value="" disabled>
              Select Tech Background
            </option>
            {TechBackgroundSchema.options.map((option) => (
              <option key={option} value={option}>
                {option.replace('_', ' ')}
              </option>
            ))}
          </select>
          <div
            className={`absolute inset-y-0 right-0 pr-3 flex items-center pointer-events-none ${
              isDark ? 'text-slate-500' : 'text-slate-400'
            }`}
          >
            <ChevronRight className="w-5 h-5 rotate-90" />
          </div>
        </div>
      </div>

      {/* Navigation buttons */}
      <div className="flex gap-4">
        <button
          type="button"
          onClick={onBack}
          className={`flex-1 flex items-center justify-center gap-2 py-3 px-4 rounded-lg text-sm font-semibold transition-all duration-200 ${
            isDark
              ? 'bg-slate-800 hover:bg-slate-700 text-slate-300 border border-slate-700'
              : 'bg-slate-100 hover:bg-slate-200 text-slate-700 border border-slate-200'
          } focus:outline-none focus:ring-2 focus:ring-slate-500 focus:ring-offset-2 ${
            isDark ? 'focus:ring-offset-slate-900' : 'focus:ring-offset-white'
          }`}
        >
          <ChevronLeft className="w-4 h-4" />
          Back
        </button>
        <button
          type="submit"
          className={`flex-1 flex items-center justify-center gap-2 py-3 px-4 rounded-lg text-sm font-semibold transition-all duration-200 ${
            isDark
              ? 'bg-emerald-500 hover:bg-emerald-400 text-white shadow-lg shadow-emerald-500/20'
              : 'bg-emerald-600 hover:bg-emerald-500 text-white shadow-lg shadow-emerald-600/20'
          } focus:outline-none focus:ring-2 focus:ring-emerald-500 focus:ring-offset-2 ${
            isDark ? 'focus:ring-offset-slate-900' : 'focus:ring-offset-white'
          }`}
        >
          Next
          <ChevronRight className="w-4 h-4" />
        </button>
      </div>
    </form>
  );
};

export default ProfileWizardStep2;
