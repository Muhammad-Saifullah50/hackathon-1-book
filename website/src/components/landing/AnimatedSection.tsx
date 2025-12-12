import { type ReactNode } from 'react';
import { motion, useReducedMotion } from 'framer-motion';

interface AnimatedSectionProps {
  children: ReactNode;
  className?: string;
  /** Delay before animation starts (in seconds) */
  delay?: number;
}

/**
 * Wrapper component for scroll-triggered fade-in-up animation.
 * Respects user's reduced motion preferences for accessibility.
 */
export function AnimatedSection({
  children,
  className = '',
  delay = 0,
}: AnimatedSectionProps) {
  const shouldReduceMotion = useReducedMotion();

  // If user prefers reduced motion, render without animation
  if (shouldReduceMotion) {
    return <div className={className}>{children}</div>;
  }

  return (
    <motion.div
      className={className}
      initial={{ opacity: 0, y: 20 }}
      whileInView={{ opacity: 1, y: 0 }}
      transition={{
        duration: 0.5,
        delay,
        ease: 'easeOut',
      }}
      viewport={{ once: true, margin: '-50px' }}
    >
      {children}
    </motion.div>
  );
}

export default AnimatedSection;
