import { type LucideIcon } from 'lucide-react';
import { clsx } from 'clsx';
import { useSafeColorMode } from '../../hooks/useSafeColorMode';

interface CurriculumCardProps {
  /** Lucide icon component */
  icon: LucideIcon;
  /** Card title */
  title: string;
  /** Card subtitle/description */
  subtitle: string;
  /** Week range (e.g., "Weeks 1-5") */
  weekRange: string;
  /** Optional custom class names */
  className?: string;
}

/**
 * Glassmorphism card component for displaying curriculum modules.
 * Features hover effects and subtle animations.
 * Supports light/dark mode theming.
 */
export function CurriculumCard({
  icon: Icon,
  title,
  subtitle,
  weekRange,
  className = '',
}: CurriculumCardProps) {
  const { isDark } = useSafeColorMode();

  return (
    <div
      className={clsx(
        // Base glassmorphism styles
        isDark ? 'bg-slate-900/50' : 'bg-white/80',
        'backdrop-blur-md',
        isDark ? 'border border-white/10' : 'border border-slate-200',
        'rounded-2xl',
        // Padding and spacing
        'p-6 lg:p-8',
        // Hover effects
        'transition-all duration-300',
        isDark
          ? 'hover:bg-slate-800/60 hover:border-white/20 hover:shadow-lg hover:shadow-emerald-500/10'
          : 'hover:bg-white hover:border-emerald-200 hover:shadow-lg hover:shadow-emerald-500/15',
        'hover:-translate-y-1',
        // Focus states for accessibility
        'focus-within:ring-2 focus-within:ring-emerald-500/50',
        className
      )}
    >
      {/* Icon container with gradient background */}
      <div className="mb-6">
        <div
          className={clsx(
            'inline-flex items-center justify-center',
            'w-14 h-14 rounded-xl',
            isDark
              ? 'bg-gradient-to-br from-emerald-500/15 to-amber-500/15 border border-emerald-500/25'
              : 'bg-gradient-to-br from-emerald-100 to-amber-100 border border-emerald-200'
          )}
        >
          <Icon
            className={`w-7 h-7 ${isDark ? 'text-emerald-400' : 'text-emerald-600'}`}
            strokeWidth={1.5}
          />
        </div>
      </div>

      {/* Week range badge */}
      <span
        className={clsx(
          'inline-block mb-4',
          'px-3 py-1 rounded-full',
          'text-xs font-medium tracking-wide',
          isDark
            ? 'bg-amber-500/15 text-amber-400 border border-amber-500/25'
            : 'bg-amber-100 text-amber-700 border border-amber-200'
        )}
      >
        {weekRange}
      </span>

      {/* Title */}
      <h3
        className={`text-xl font-semibold tracking-tight mb-2 ${
          isDark ? 'text-slate-50' : 'text-slate-900'
        }`}
      >
        {title}
      </h3>

      {/* Subtitle */}
      <p className={`text-sm leading-relaxed ${isDark ? 'text-slate-400' : 'text-slate-600'}`}>
        {subtitle}
      </p>
    </div>
  );
}

export default CurriculumCard;
