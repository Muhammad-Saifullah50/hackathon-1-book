import { ReactNode } from 'react';

export interface CardProps {
  title?: string;
  icon?: ReactNode;
  children: ReactNode;
  className?: string;
}

export interface AlertProps {
  variant?: 'default' | 'destructive';
  title?: string;
  children: ReactNode;
}

export interface BadgeProps {
  variant?: 'default' | 'secondary' | 'outline';
  children: ReactNode;
}
