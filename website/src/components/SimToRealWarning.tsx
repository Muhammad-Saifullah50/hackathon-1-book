import React from 'react';
import { AlertTriangle } from 'lucide-react';

interface SimToRealWarningProps {
  children: React.ReactNode;
  title?: string;
}

export function SimToRealWarning({ children, title = "Sim-to-Real Warning" }: SimToRealWarningProps) {
  return (
    <div className="my-6 rounded-lg border-2 border-orange-500/50 bg-gradient-to-br from-orange-50 to-amber-50 dark:from-orange-950/30 dark:to-amber-950/30 dark:border-orange-500/70 overflow-hidden shadow-lg">
      {/* Header with icon */}
      <div className="flex items-center gap-3 px-4 py-3 bg-orange-500/10 dark:bg-orange-500/20 border-b border-orange-500/30">
        <div className="flex-shrink-0 w-10 h-10 rounded-full bg-orange-500 dark:bg-orange-600 flex items-center justify-center shadow-md">
          <AlertTriangle className="w-5 h-5 text-white" />
        </div>
        <h4 className="text-lg font-bold text-orange-900 dark:text-orange-100 m-0">
          {title}
        </h4>
      </div>

      {/* Content */}
      <div className="px-4 py-4 text-gray-800 dark:text-gray-200">
        {children}
      </div>

      {/* Bottom accent bar */}
      <div className="h-1 bg-gradient-to-r from-orange-500 via-amber-500 to-orange-500"></div>
    </div>
  );
}
