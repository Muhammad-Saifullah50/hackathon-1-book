import React from 'react';
import { useSafeColorMode } from '../../hooks/useSafeColorMode';

interface LogoProps {
  size?: number;
  className?: string;
  showText?: boolean;
}

export function Logo({ size = 32, className = '', showText = false }: LogoProps) {
  const { isDark } = useSafeColorMode();

  // Colors based on theme
  const primaryColor = isDark ? '#34d399' : '#059669'; // emerald-400 / emerald-600
  const secondaryColor = isDark ? '#fbbf24' : '#d97706'; // amber-400 / amber-600
  const bgColor = isDark ? '#0f172a' : '#f8fafc'; // slate-900 / slate-50

  return (
    <div className={`flex items-center gap-2 ${className}`}>
      <svg
        width={size}
        height={size}
        viewBox="0 0 48 48"
        fill="none"
        xmlns="http://www.w3.org/2000/svg"
        aria-label="RoboLearn Logo"
      >
        {/* Background circle */}
        <circle cx="24" cy="24" r="22" fill={bgColor} stroke={primaryColor} strokeWidth="2" />

        {/* Robot head outline */}
        <rect x="14" y="12" width="20" height="18" rx="4" fill={primaryColor} fillOpacity="0.15" stroke={primaryColor} strokeWidth="2" />

        {/* Robot eyes */}
        <circle cx="19" cy="20" r="3" fill={primaryColor} />
        <circle cx="29" cy="20" r="3" fill={primaryColor} />

        {/* Eye highlights */}
        <circle cx="18" cy="19" r="1" fill="white" fillOpacity="0.8" />
        <circle cx="28" cy="19" r="1" fill="white" fillOpacity="0.8" />

        {/* Robot mouth/smile */}
        <path d="M18 26 C20 28, 28 28, 30 26" stroke={primaryColor} strokeWidth="2" strokeLinecap="round" fill="none" />

        {/* Antenna */}
        <line x1="24" y1="12" x2="24" y2="6" stroke={secondaryColor} strokeWidth="2" strokeLinecap="round" />
        <circle cx="24" cy="5" r="2" fill={secondaryColor} />

        {/* Neural network dots */}
        <circle cx="10" cy="24" r="1.5" fill={secondaryColor} fillOpacity="0.6" />
        <circle cx="38" cy="24" r="1.5" fill={secondaryColor} fillOpacity="0.6" />
        <circle cx="24" cy="38" r="1.5" fill={secondaryColor} fillOpacity="0.6" />

        {/* Connection lines */}
        <line x1="12" y1="24" x2="14" y2="21" stroke={secondaryColor} strokeWidth="1" strokeOpacity="0.4" />
        <line x1="36" y1="24" x2="34" y2="21" stroke={secondaryColor} strokeWidth="1" strokeOpacity="0.4" />
        <line x1="24" y1="36" x2="24" y2="30" stroke={secondaryColor} strokeWidth="1" strokeOpacity="0.4" />
      </svg>

      {showText && (
        <span className={`font-bold text-lg tracking-tight ${
          isDark ? 'text-slate-100' : 'text-slate-900'
        }`}>
          <span style={{ color: primaryColor }}>Robo</span>
          <span>Learn</span>
        </span>
      )}
    </div>
  );
}

export function LogoMark({ size = 24, className = '' }: Omit<LogoProps, 'showText'>) {
  return <Logo size={size} className={className} showText={false} />;
}

export default Logo;
