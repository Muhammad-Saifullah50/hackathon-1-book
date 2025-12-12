import { type LucideIcon } from 'lucide-react';
import { clsx } from 'clsx';
import { useSafeColorMode } from '../../hooks/useSafeColorMode';

interface HardwareItemProps {
  /** Lucide icon component */
  icon: LucideIcon;
  /** Hardware category (e.g., "Brain", "Eyes", "Body") */
  category: string;
  /** Hardware name (e.g., "NVIDIA Jetson Orin") */
  name: string;
  /** Hardware description */
  description: string;
  /** Whether this item is optional */
  isOptional?: boolean;
  /** Optional custom class names */
  className?: string;
}

/**
 * Game-style hardware item card for the Lab section.
 * Displays hardware requirements with a tech aesthetic.
 * Supports light/dark mode theming.
 */
export function HardwareItem({
  icon: Icon,
  category,
  name,
  description,
  isOptional = false,
  className = '',
}: HardwareItemProps) {
  const { isDark } = useSafeColorMode();

  return (
    <div
      className={clsx(
        // Base styles - game loadout aesthetic
        'relative backdrop-blur-sm',
        isDark ? 'bg-slate-900/60' : 'bg-white/80',
        isDark ? 'border border-white/10' : 'border border-slate-200',
        'rounded-xl',
        'p-5 lg:p-6',
        // Hover effects
        'transition-all duration-300',
        isDark
          ? 'hover:bg-slate-800/60 hover:border-emerald-500/30 hover:shadow-lg hover:shadow-emerald-500/5'
          : 'hover:bg-white hover:border-emerald-300 hover:shadow-lg hover:shadow-emerald-500/10',
        // Tech corner accents
        'before:absolute before:top-0 before:left-0 before:w-4 before:h-4',
        isDark
          ? 'before:border-t-2 before:border-l-2 before:border-emerald-500/40 before:rounded-tl-xl'
          : 'before:border-t-2 before:border-l-2 before:border-emerald-400/60 before:rounded-tl-xl',
        'after:absolute after:bottom-0 after:right-0 after:w-4 after:h-4',
        isDark
          ? 'after:border-b-2 after:border-r-2 after:border-amber-500/40 after:rounded-br-xl'
          : 'after:border-b-2 after:border-r-2 after:border-amber-400/60 after:rounded-br-xl',
        className
      )}
    >
      {/* Category badge */}
      <div className="flex items-center justify-between mb-4">
        <span
          className={clsx(
            'inline-flex items-center gap-2 px-3 py-1 rounded-full',
            'text-xs font-bold tracking-wider uppercase',
            isDark
              ? 'bg-emerald-500/10 text-emerald-400 border border-emerald-500/20'
              : 'bg-emerald-100 text-emerald-700 border border-emerald-200'
          )}
        >
          <Icon className="w-3 h-3" />
          {category}
        </span>

        {isOptional && (
          <span className={`text-xs font-medium ${isDark ? 'text-slate-500' : 'text-slate-500'}`}>
            OPTIONAL
          </span>
        )}
      </div>

      {/* Hardware name */}
      <h4 className={`text-lg font-semibold mb-2 ${isDark ? 'text-slate-50' : 'text-slate-900'}`}>
        {name}
      </h4>

      {/* Description */}
      <p className={`text-sm leading-relaxed ${isDark ? 'text-slate-400' : 'text-slate-600'}`}>
        {description}
      </p>

      {/* Status indicator */}
      <div className="mt-4 flex items-center gap-2">
        <span
          className={clsx(
            'w-2 h-2 rounded-full',
            isOptional ? 'bg-yellow-400' : 'bg-green-400'
          )}
        />
        <span className={`text-xs ${isDark ? 'text-slate-500' : 'text-slate-500'}`}>
          {isOptional ? 'Cloud alternative available' : 'Required'}
        </span>
      </div>
    </div>
  );
}

export default HardwareItem;
