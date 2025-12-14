// website/src/components/profile/ProfileWizardStep3.tsx
import React, { useState } from 'react';
import { UserProfileUpdate, PrimaryGoalSchema, LearningModeSchema, LearningSpeedSchema } from '../../data/profile-schema';
import { useSafeColorMode } from '../../hooks/useSafeColorMode';
import { z } from 'zod';
import { ChevronLeft, AlertCircle, Target, BookOpen, Zap, Clock, CheckCircle } from 'lucide-react';
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '../ui/select';

interface Props {
  onSubmit: (data: Partial<UserProfileUpdate>) => void;
  onBack: () => void;
  initialData: Partial<UserProfileUpdate>;
}

const ProfileWizardStep3: React.FC<Props> = ({ onSubmit, onBack, initialData }) => {
  const { isDark } = useSafeColorMode();
  const [primaryGoal, setPrimaryGoal] = useState(initialData.primary_goal || '');
  const [learningMode, setLearningMode] = useState(initialData.learning_mode || '');
  const [learningSpeed, setLearningSpeed] = useState(initialData.learning_speed || '');
  const [timePerWeek, setTimePerWeek] = useState(initialData.time_per_week || 5);
  const [error, setError] = useState<string | null>(null);

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    try {
      // Validate with Zod
      PrimaryGoalSchema.parse(primaryGoal);
      LearningModeSchema.parse(learningMode);
      LearningSpeedSchema.parse(learningSpeed);
      z.number().int().nonnegative().parse(timePerWeek);

      onSubmit({
        primary_goal: primaryGoal as any,
        learning_mode: learningMode as any,
        learning_speed: learningSpeed as any,
        time_per_week: Number(timePerWeek),
      });
    } catch (e) {
      if (e instanceof z.ZodError) {
        setError(e.issues[0].message);
      } else {
        setError('An unexpected error occurred.');
      }
    }
  };

  const formatOption = (option: string) => {
    return option.replace(/_/g, ' ');
  };

  const inputClassName = `block w-full pl-10 pr-4 py-3 rounded-lg transition-colors ${
    isDark
      ? 'bg-slate-900 border-slate-700 text-slate-200 placeholder-slate-500 focus:border-emerald-500 focus:ring-1 focus:ring-emerald-500'
      : 'bg-white border-slate-300 text-slate-900 placeholder-slate-400 focus:border-emerald-500 focus:ring-1 focus:ring-emerald-500'
  } border outline-none`;

  return (
    <form onSubmit={handleSubmit} className="space-y-6">
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
            3
          </div>
          <h2
            className={`text-xl font-semibold tracking-tight ${
              isDark ? 'text-slate-100' : 'text-slate-900'
            }`}
          >
            Your Strategy
          </h2>
        </div>
        <p className={`text-sm ${isDark ? 'text-slate-400' : 'text-slate-600'}`}>
          Let us know your goals and preferred learning style.
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

      {/* Primary Goal field */}
      <div className="space-y-2">
        <label
          htmlFor="primary_goal"
          className={`block text-sm font-medium ${
            isDark ? 'text-slate-300' : 'text-slate-700'
          }`}
        >
          Primary Goal
        </label>
        <div className="relative">
          <div
            className={`absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none z-10 ${
              isDark ? 'text-slate-500' : 'text-slate-400'
            }`}
          >
            <Target className="w-5 h-5" />
          </div>
          <Select value={primaryGoal} onValueChange={setPrimaryGoal}>
            <SelectTrigger className="pl-10 capitalize">
              <SelectValue placeholder="Select Primary Goal" />
            </SelectTrigger>
            <SelectContent>
              {PrimaryGoalSchema.options.map((option) => (
               <SelectItem className='capitalize' key={option} value={option}>
                  {formatOption(option)}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
        </div>
      </div>

      {/* Learning Mode field */}
      <div className="space-y-2">
        <label
          htmlFor="learning_mode"
          className={`block text-sm font-medium ${
            isDark ? 'text-slate-300' : 'text-slate-700'
          }`}
        >
          Learning Mode
        </label>
        <div className="relative">
          <div
            className={`absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none z-10 ${
              isDark ? 'text-slate-500' : 'text-slate-400'
            }`}
          >
            <BookOpen className="w-5 h-5" />
          </div>
          <Select value={learningMode} onValueChange={setLearningMode}>
            <SelectTrigger className="pl-10 capitalize">
              <SelectValue placeholder="Select Learning Mode" />
            </SelectTrigger>
            <SelectContent>
              {LearningModeSchema.options.map((option) => (
                <SelectItem className='capitalize' key={option} value={option}>
                  {formatOption(option)}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
        </div>
      </div>

      {/* Learning Speed field */}
      <div className="space-y-2">
        <label
          htmlFor="learning_speed"
          className={`block text-sm font-medium ${
            isDark ? 'text-slate-300' : 'text-slate-700'
          }`}
        >
          Learning Speed
        </label>
        <div className="relative">
          <div
            className={`absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none z-10 ${
              isDark ? 'text-slate-500' : 'text-slate-400'
            }`}
          >
            <Zap className="w-5 h-5" />
          </div>
          <Select value={learningSpeed} onValueChange={setLearningSpeed}>
            <SelectTrigger className="pl-10 capitalize">
              <SelectValue className='capitalize' placeholder="Select Learning Speed" />
            </SelectTrigger>
            <SelectContent>
              {LearningSpeedSchema.options.map((option) => (
                <SelectItem className='capitalize' key={option} value={option}>
                  {formatOption(option)}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
        </div>
      </div>

      {/* Time Per Week field */}
      <div className="space-y-2">
        <label
          htmlFor="time_per_week"
          className={`block text-sm font-medium ${
            isDark ? 'text-slate-300' : 'text-slate-700'
          }`}
        >
          Time Per Week (hours)
        </label>
        <div className="relative">
          <div
            className={`absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none ${
              isDark ? 'text-slate-500' : 'text-slate-400'
            }`}
          >
            <Clock className="w-5 h-5" />
          </div>
          <input
            type="number"
            id="time_per_week"
            value={timePerWeek}
            onChange={(e) => setTimePerWeek(Number(e.target.value))}
            min="1"
            required
            className={inputClassName}
          />
        </div>
      </div>

      {/* Navigation buttons */}
      <div className="flex gap-4 pt-2">
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
          <CheckCircle className="w-4 h-4" />
          Create Profile
        </button>
      </div>
    </form>
  );
};

export default ProfileWizardStep3;
