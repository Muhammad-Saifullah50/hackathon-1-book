import { render, screen } from '@testing-library/react';
import { AnimatedSection } from '@/components/landing/AnimatedSection';

// Mock framer-motion
jest.mock('framer-motion', () => ({
  motion: {
    div: ({ children, className, ...props }: React.HTMLAttributes<HTMLDivElement>) => (
      <div className={className} data-testid="motion-div" {...props}>
        {children}
      </div>
    ),
  },
  useReducedMotion: jest.fn(() => false),
}));

describe('AnimatedSection', () => {
  it('renders children correctly', () => {
    render(
      <AnimatedSection>
        <p>Test content</p>
      </AnimatedSection>
    );

    expect(screen.getByText('Test content')).toBeInTheDocument();
  });

  it('applies custom className', () => {
    render(
      <AnimatedSection className="custom-class">
        <p>Content</p>
      </AnimatedSection>
    );

    const container = screen.getByTestId('motion-div');
    expect(container).toHaveClass('custom-class');
  });

  it('renders without animation when reduced motion is preferred', () => {
    const { useReducedMotion } = require('framer-motion');
    useReducedMotion.mockReturnValue(true);

    render(
      <AnimatedSection className="test-class">
        <p>Reduced motion content</p>
      </AnimatedSection>
    );

    expect(screen.getByText('Reduced motion content')).toBeInTheDocument();
    // Should render a plain div, not motion.div
    expect(screen.queryByTestId('motion-div')).not.toBeInTheDocument();
  });

  it('accepts delay prop', () => {
    render(
      <AnimatedSection delay={0.5}>
        <p>Delayed content</p>
      </AnimatedSection>
    );

    expect(screen.getByText('Delayed content')).toBeInTheDocument();
  });
});
