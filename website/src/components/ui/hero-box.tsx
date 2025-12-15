import React from 'react';
import { useSafeColorMode } from '../../hooks/useSafeColorMode';

interface HeroBoxProps {
  title: string;
  children: React.ReactNode;
  /** Color variant for the hero box - controls gradient colors */
  variant?: 'blue' | 'green' | 'purple' | 'amber' | 'emerald' | 'cyan' | 'rose';
}

/**
 * HeroBox - A themed hero section component for documentation pages
 *
 * Renders a prominent callout box with a title and content, properly
 * themed for both light and dark modes using the project's color system.
 *
 * @example
 * <HeroBox title="Why does a robot need to see?" variant="blue">
 *   <p>Welcome to the frontier of Physical AI...</p>
 * </HeroBox>
 */
const HeroBox: React.FC<HeroBoxProps> = ({ title, children, variant = 'blue' }) => {
  const { isDark } = useSafeColorMode();

  // Define color schemes for each variant
  const colorSchemes = {
    blue: {
      light: 'from-blue-50 to-indigo-50 border-blue-200',
      dark: 'from-blue-950/40 to-indigo-950/40 border-blue-500/30',
      titleLight: 'text-blue-900',
      titleDark: 'text-blue-100',
    },
    green: {
      light: 'from-green-50 to-emerald-50 border-green-200',
      dark: 'from-green-950/40 to-emerald-950/40 border-green-500/30',
      titleLight: 'text-green-900',
      titleDark: 'text-green-100',
    },
    emerald: {
      light: 'from-emerald-50 to-teal-50 border-emerald-200',
      dark: 'from-emerald-950/40 to-teal-950/40 border-emerald-500/30',
      titleLight: 'text-emerald-900',
      titleDark: 'text-emerald-100',
    },
    purple: {
      light: 'from-purple-50 to-violet-50 border-purple-200',
      dark: 'from-purple-950/40 to-violet-950/40 border-purple-500/30',
      titleLight: 'text-purple-900',
      titleDark: 'text-purple-100',
    },
    amber: {
      light: 'from-amber-50 to-orange-50 border-amber-200',
      dark: 'from-amber-950/40 to-orange-950/40 border-amber-500/30',
      titleLight: 'text-amber-900',
      titleDark: 'text-amber-100',
    },
    cyan: {
      light: 'from-cyan-50 to-sky-50 border-cyan-200',
      dark: 'from-cyan-950/40 to-sky-950/40 border-cyan-500/30',
      titleLight: 'text-cyan-900',
      titleDark: 'text-cyan-100',
    },
    rose: {
      light: 'from-rose-50 to-pink-50 border-rose-200',
      dark: 'from-rose-950/40 to-pink-950/40 border-rose-500/30',
      titleLight: 'text-rose-900',
      titleDark: 'text-rose-100',
    },
  };

  const scheme = colorSchemes[variant];

  const gradientClasses = isDark ? scheme.dark : scheme.light;
  const titleColor = isDark ? scheme.titleDark : scheme.titleLight;
  const textColor = isDark ? 'text-slate-300' : 'text-slate-700';

  return (
    <div
      className={`shadow-lg rounded-lg p-6 mb-8 bg-gradient-to-r ${gradientClasses} border`}
    >
      <h2 className={`text-2xl font-bold mb-4 ${titleColor}`}>{title}</h2>
      <div className={`text-lg ${textColor}`}>{children}</div>
    </div>
  );
};

export default HeroBox;
