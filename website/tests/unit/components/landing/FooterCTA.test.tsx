import { render, screen } from '@testing-library/react';
import { FooterCTA } from '@/components/landing/FooterCTA';

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

describe('FooterCTA', () => {
  it('renders the main headline', () => {
    render(<FooterCTA />);

    expect(screen.getByText(/Build the/i)).toBeInTheDocument();
    expect(screen.getByText(/Future/i)).toBeInTheDocument();
  });

  it('renders the gradient text on "Future"', () => {
    render(<FooterCTA />);

    const futureText = screen.getByText(/Future/i);
    expect(futureText).toHaveClass('bg-gradient-to-r');
    expect(futureText).toHaveClass('from-cyan-400');
    expect(futureText).toHaveClass('to-violet-600');
  });

  it('renders the CTA button linking to signup', () => {
    render(<FooterCTA />);

    const ctaButton = screen.getByRole('link', { name: /Create Free Profile/i });
    expect(ctaButton).toHaveAttribute('href', '/signup');
  });

  it('renders the Panaversity badge', () => {
    render(<FooterCTA />);

    expect(screen.getByText(/Powered by Panaversity/i)).toBeInTheDocument();
  });

  it('renders the Gemini badge', () => {
    render(<FooterCTA />);

    expect(screen.getByText(/Built with Gemini/i)).toBeInTheDocument();
  });

  it('has gradient background', () => {
    const { container } = render(<FooterCTA />);

    const gradientDiv = container.querySelector('.bg-gradient-to-t');
    expect(gradientDiv).toBeInTheDocument();
  });

  it('renders description text', () => {
    render(<FooterCTA />);

    expect(
      screen.getByText(/Join thousands of learners/i)
    ).toBeInTheDocument();
  });
});
