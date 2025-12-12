import { type ReactNode } from 'react';
import { clsx } from 'clsx';
import { useSafeColorMode } from '../../hooks/useSafeColorMode';

interface FeatureDemoProps {
  /** Feature title */
  title: string;
  /** Feature description */
  description: string;
  /** Visual component (ChatbotDemo or PersonalizationDemo) */
  visual: ReactNode;
  /** Reverse layout for zig-zag pattern */
  reversed?: boolean;
  /** Optional custom class names */
  className?: string;
}

/**
 * Container component for feature demos with zig-zag layout support.
 * Shows text on one side and visual demo on the other.
 * Supports light/dark mode theming.
 */
export function FeatureDemo({
  title,
  description,
  visual,
  reversed = false,
  className = '',
}: FeatureDemoProps) {
  const { isDark } = useSafeColorMode();

  return (
    <div
      className={clsx(
        'grid grid-cols-1 lg:grid-cols-2 gap-8 lg:gap-16 items-center',
        className
      )}
    >
      {/* Text content */}
      <div
        className={clsx(
          'space-y-4 text-center lg:text-left',
          reversed && 'lg:order-2'
        )}
      >
        <h3
          className={`text-2xl sm:text-3xl lg:text-4xl font-bold tracking-tight ${
            isDark ? 'text-slate-50' : 'text-slate-900'
          }`}
        >
          {title}
        </h3>
        <p
          className={`text-lg max-w-md mx-auto lg:mx-0 ${
            isDark ? 'text-slate-400' : 'text-slate-600'
          }`}
        >
          {description}
        </p>
      </div>

      {/* Visual demo */}
      <div className={clsx(reversed && 'lg:order-1')}>{visual}</div>
    </div>
  );
}

export default FeatureDemo;
