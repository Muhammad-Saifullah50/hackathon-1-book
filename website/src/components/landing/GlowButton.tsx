import { type ReactNode } from 'react';
import Link from '@docusaurus/Link';
import { clsx } from 'clsx';
import { useSafeColorMode } from '../../hooks/useSafeColorMode';

type ButtonVariant = 'primary' | 'secondary';

interface GlowButtonProps {
  children: ReactNode;
  href: string;
  variant?: ButtonVariant;
  className?: string;
  /** External link opens in new tab */
  external?: boolean;
}

/**
 * Stylized button component with glow effects for the landing page.
 * Uses Docusaurus Link for internal navigation.
 * Supports light/dark mode theming.
 */
export function GlowButton({
  children,
  href,
  variant = 'primary',
  className = '',
  external = false,
}: GlowButtonProps) {
  const { isDark } = useSafeColorMode();

  const variantStyles: Record<ButtonVariant, string> = {
    primary: isDark
      ? clsx(
          'bg-emerald-500 text-white font-semibold',
          'shadow-lg shadow-emerald-500/25',
          'hover:bg-emerald-400 hover:shadow-emerald-500/40',
          'active:bg-emerald-600'
        )
      : clsx(
          'bg-emerald-600 text-white font-semibold',
          'shadow-lg shadow-emerald-600/25',
          'hover:bg-emerald-500 hover:shadow-emerald-600/40',
          'active:bg-emerald-700'
        ),
    secondary: isDark
      ? clsx(
          'bg-transparent border-2 border-slate-600 text-slate-200',
          'hover:bg-slate-800 hover:border-slate-500',
          'active:bg-slate-700'
        )
      : clsx(
          'bg-transparent border-2 border-slate-300 text-slate-700',
          'hover:bg-slate-100 hover:border-slate-400',
          'active:bg-slate-200'
        ),
  };

  const baseStyles = clsx(
    'inline-flex items-center justify-center',
    'px-8 py-4 rounded-lg',
    'text-base tracking-tight',
    'transition-all duration-200',
    isDark
      ? 'focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-emerald-400 focus-visible:ring-offset-2 focus-visible:ring-offset-slate-950'
      : 'focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-emerald-500 focus-visible:ring-offset-2 focus-visible:ring-offset-white'
  );

  const combinedStyles = clsx(baseStyles, variantStyles[variant], className);

  if (external) {
    return (
      <a
        href={href}
        className={combinedStyles}
        target="_blank"
        rel="noopener noreferrer"
      >
        {children}
      </a>
    );
  }

  return (
    <Link to={href} className={combinedStyles}>
      {children}
    </Link>
  );
}

export default GlowButton;
