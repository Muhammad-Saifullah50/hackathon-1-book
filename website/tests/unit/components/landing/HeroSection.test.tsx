import { render, screen } from '@testing-library/react';
import { HeroSection } from '@/components/landing/HeroSection';

// Mock framer-motion
jest.mock('framer-motion', () => ({
  motion: {
    div: ({ children, className, ...props }: React.HTMLAttributes<HTMLDivElement>) => (
      <div className={className} {...props}>
        {children}
      </div>
    ),
  },
  useReducedMotion: jest.fn(() => false),
}));

describe('HeroSection', () => {
  it('renders the main headline', () => {
    render(<HeroSection />);

    expect(screen.getByText(/Wake Up the/i)).toBeInTheDocument();
    expect(screen.getByText(/Metal\./i)).toBeInTheDocument();
  });

  it('renders the gradient text on "Metal"', () => {
    render(<HeroSection />);

    const metalText = screen.getByText(/Metal\./i);
    expect(metalText).toHaveClass('bg-gradient-to-r');
    expect(metalText).toHaveClass('from-cyan-400');
    expect(metalText).toHaveClass('to-violet-600');
  });

  it('renders the subheadline', () => {
    render(<HeroSection />);

    expect(
      screen.getByText(/From Python code to Walking Robots/i)
    ).toBeInTheDocument();
  });

  it('renders the primary CTA linking to docs', () => {
    render(<HeroSection />);

    const startReadingLink = screen.getByRole('link', { name: /Start Reading/i });
    expect(startReadingLink).toHaveAttribute('href', '/docs/module-01/overview');
  });

  it('renders the secondary CTA', () => {
    render(<HeroSection />);

    const watchDemoLink = screen.getByRole('link', { name: /Watch Demo/i });
    expect(watchDemoLink).toHaveAttribute('href', '#curriculum');
  });

  it('renders the robot image with proper alt text', () => {
    render(<HeroSection />);

    const robotImage = screen.getByAltText(/Physical AI Robot/i);
    expect(robotImage).toBeInTheDocument();
    expect(robotImage).toHaveAttribute('src', '/img/landing/robot-hero.svg');
  });

  it('renders trust indicators', () => {
    render(<HeroSection />);

    expect(screen.getByText(/13 Week Curriculum/i)).toBeInTheDocument();
    expect(screen.getByText(/Hands-on Projects/i)).toBeInTheDocument();
    expect(screen.getByText(/Free & Open Source/i)).toBeInTheDocument();
  });

  it('is responsive with proper container structure', () => {
    render(<HeroSection />);

    const section = screen.getByRole('img', { name: /Physical AI Robot/i }).closest('section');
    expect(section).toHaveClass('min-h-[90vh]');
  });
});
