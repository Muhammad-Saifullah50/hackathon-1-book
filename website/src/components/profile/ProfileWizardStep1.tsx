// website/src/components/profile/ProfileWizardStep1.tsx
import React, { useState } from 'react';
import { UserProfileUpdate, AgeRangeSchema } from '../../data/profile-schema';
import { useSafeColorMode } from '../../hooks/useSafeColorMode';
import { z } from 'zod';
import { ChevronRight, AlertCircle, Calendar } from 'lucide-react';
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '../ui/select';

interface Props {
  onNext: (data: Partial<UserProfileUpdate>) => void;
  initialData: Partial<UserProfileUpdate>;
}

const ProfileWizardStep1: React.FC<Props> = ({ onNext, initialData }) => {
  const { isDark } = useSafeColorMode();
  const [ageRange, setAgeRange] = useState(initialData.age_range || '');
  const [error, setError] = useState<string | null>(null);

  const handleNext = (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    try {
      // Validate with Zod
      AgeRangeSchema.parse(ageRange);
      onNext({ age_range: ageRange as any });
    } catch (e) {
      if (e instanceof z.ZodError) {
        // ZodError contains an array of issues
        setError(e.issues[0].message);
      } else {
        setError('An unexpected error occurred.');
      }
    }
  };

  const formatOption = (option: string) => {
    return option.replace('_', ' ').replace('plus', '+');
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
            1
          </div>
          <h2
            className={`text-xl font-semibold tracking-tight ${
              isDark ? 'text-slate-100' : 'text-slate-900'
            }`}
          >
            The Basics
          </h2>
        </div>
        <p className={`text-sm ${isDark ? 'text-slate-400' : 'text-slate-600'}`}>
          Help us understand your background so we can personalize your learning journey.
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

      {/* Age Range field */}
      <div className="space-y-2">
        <label
          htmlFor="age_range"
          className={`block text-sm font-medium ${
            isDark ? 'text-slate-300' : 'text-slate-700'
          }`}
        >
          Age Range
        </label>
        <div className="relative">
          <div
            className={`absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none z-10 ${
              isDark ? 'text-slate-500' : 'text-slate-400'
            }`}
          >
            <Calendar className="w-5 h-5" />
          </div>
          <Select value={ageRange} onValueChange={setAgeRange}>
            <SelectTrigger className="pl-10">
              <SelectValue placeholder="Select Age Range" />
            </SelectTrigger>
            <SelectContent>
              {AgeRangeSchema.options.map((option) => (
                <SelectItem key={option} value={option}>
                  {formatOption(option)}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
        </div>
      </div>

      {/* Submit button */}
      <button
        type="submit"
        className={`w-full flex items-center justify-center gap-2 py-3 px-4 rounded-lg text-sm font-semibold transition-all duration-200 ${
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
    </form>
  );
};

export default ProfileWizardStep1;
