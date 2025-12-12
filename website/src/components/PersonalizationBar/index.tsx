import React, { useState } from 'react';
import { Badge } from '../ui/badge';
import { useSafeColorMode } from '../../hooks/useSafeColorMode';
import { Code2, Cpu, Sparkles } from 'lucide-react';

export default function PersonalizationBar() {
  const [role, setRole] = useState<'Software' | 'Hardware' | null>(null);
  const { isDark } = useSafeColorMode();

  return (
    <div
      className={`my-6 p-5 rounded-xl ${
        isDark
          ? 'bg-slate-900/60 border border-white/10'
          : 'bg-white border border-slate-200 shadow-sm'
      }`}
    >
      <div className="flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4">
        <div className="flex items-start gap-3">
          <div
            className={`flex items-center justify-center w-10 h-10 rounded-lg flex-shrink-0 ${
              isDark ? 'bg-emerald-500/10' : 'bg-emerald-100'
            }`}
          >
            <Sparkles
              className={`w-5 h-5 ${isDark ? 'text-emerald-400' : 'text-emerald-600'}`}
            />
          </div>
          <div>
            <h4
              className={`text-lg font-semibold mb-1 ${
                isDark ? 'text-slate-100' : 'text-slate-900'
              }`}
            >
              Tailor Your Experience
            </h4>
            <p className={`text-sm ${isDark ? 'text-slate-400' : 'text-slate-600'}`}>
              Select your background to see relevant tips and code focus.
            </p>
          </div>
        </div>
        <div className="flex gap-2">
          <button
            onClick={() => setRole('Software')}
            className={`flex items-center gap-2 px-4 py-2 rounded-lg text-sm font-medium transition-all duration-200 ${
              role === 'Software'
                ? isDark
                  ? 'bg-emerald-500 text-white shadow-lg shadow-emerald-500/20'
                  : 'bg-emerald-600 text-white shadow-lg shadow-emerald-600/20'
                : isDark
                  ? 'bg-slate-800 text-slate-300 hover:bg-slate-700 border border-slate-700'
                  : 'bg-slate-100 text-slate-700 hover:bg-slate-200 border border-slate-200'
            }`}
          >
            <Code2 className="w-4 h-4" />
            Software Engineer
          </button>
          <button
            onClick={() => setRole('Hardware')}
            className={`flex items-center gap-2 px-4 py-2 rounded-lg text-sm font-medium transition-all duration-200 ${
              role === 'Hardware'
                ? isDark
                  ? 'bg-amber-500 text-white shadow-lg shadow-amber-500/20'
                  : 'bg-amber-600 text-white shadow-lg shadow-amber-600/20'
                : isDark
                  ? 'bg-slate-800 text-slate-300 hover:bg-slate-700 border border-slate-700'
                  : 'bg-slate-100 text-slate-700 hover:bg-slate-200 border border-slate-200'
            }`}
          >
            <Cpu className="w-4 h-4" />
            Hardware Engineer
          </button>
        </div>
      </div>
      {role && (
        <div
          className={`mt-4 p-4 rounded-lg border-l-4 ${
            role === 'Software'
              ? isDark
                ? 'bg-emerald-500/5 border-l-emerald-500 border border-emerald-500/10'
                : 'bg-emerald-50 border-l-emerald-500 border border-emerald-100'
              : isDark
                ? 'bg-amber-500/5 border-l-amber-500 border border-amber-500/10'
                : 'bg-amber-50 border-l-amber-500 border border-amber-100'
          }`}
        >
          <Badge
            variant="outline"
            className={`mb-2 ${
              role === 'Software'
                ? isDark
                  ? 'border-emerald-500/30 text-emerald-400'
                  : 'border-emerald-300 text-emerald-700'
                : isDark
                  ? 'border-amber-500/30 text-amber-400'
                  : 'border-amber-300 text-amber-700'
            }`}
          >
            {role === 'Software' ? (
              <Code2 className="w-3 h-3 mr-1" />
            ) : (
              <Cpu className="w-3 h-3 mr-1" />
            )}
            {role} Focus
          </Badge>
          <p className={`text-sm ${isDark ? 'text-slate-300' : 'text-slate-700'}`}>
            {role === 'Software'
              ? 'Focusing on class structures, event loops, and architectural patterns.'
              : 'Focusing on real-time constraints, frequency (Hz), and physical interfaces.'}
          </p>
        </div>
      )}
    </div>
  );
}
