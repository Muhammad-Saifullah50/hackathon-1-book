import React, { useState } from 'react';
import { Badge } from '../ui/badge';

export default function PersonalizationBar() {
  const [role, setRole] = useState<'Software' | 'Hardware' | null>(null);

  return (
    <div className="my-6 p-4 border rounded-lg bg-secondary/10">
      <div className="flex flex-col sm:flex-row items-center justify-between gap-4">
        <div>
          <h4 className="text-lg font-semibold mb-1">Tailor Your Experience</h4>
          <p className="text-sm text-muted-foreground">
            Select your background to see relevant tips and code focus.
          </p>
        </div>
        <div className="flex gap-2">
          <button
            onClick={() => setRole('Software')}
            className={`px-4 py-2 rounded-md transition-colors ${
              role === 'Software'
                ? 'bg-primary text-primary-foreground'
                : 'bg-background hover:bg-accent hover:text-accent-foreground'
            }`}
          >
            Software Engineer
          </button>
          <button
            onClick={() => setRole('Hardware')}
            className={`px-4 py-2 rounded-md transition-colors ${
              role === 'Hardware'
                ? 'bg-primary text-primary-foreground'
                : 'bg-background hover:bg-accent hover:text-accent-foreground'
            }`}
          >
            Hardware Engineer
          </button>
        </div>
      </div>
      {role && (
        <div className="mt-4 p-3 bg-background rounded border border-l-4 border-l-primary">
          <Badge variant="outline" className="mb-2">
            {role} Focus
          </Badge>
          <p className="text-sm">
            {role === 'Software'
              ? 'Focusing on class structures, event loops, and architectural patterns.'
              : 'Focusing on real-time constraints, frequency (Hz), and physical interfaces.'}
          </p>
        </div>
      )}
    </div>
  );
}
