import { render, screen } from '@testing-library/react';
import { FeaturesSection } from '@/components/landing/FeaturesSection';

// Mock framer-motion
jest.mock('framer-motion', () => ({
  motion: {
    div: ({ children, className, ...props }: React.HTMLAttributes<HTMLDivElement>) => (
      <div className={className} {...props}>
        {children}
      </div>
    ),
  },
  AnimatePresence: ({ children }: { children: React.ReactNode }) => <>{children}</>,
  useReducedMotion: jest.fn(() => true),
}));

describe('FeaturesSection', () => {
  it('renders the section title', () => {
    render(<FeaturesSection />);

    expect(screen.getByText(/Learn/i)).toBeInTheDocument();
    expect(screen.getByText(/Your Way/i)).toBeInTheDocument();
  });

  it('renders the Interactive Learning badge', () => {
    render(<FeaturesSection />);

    expect(screen.getByText('Interactive Learning')).toBeInTheDocument();
  });

  it('renders the AI Tutor feature title', () => {
    render(<FeaturesSection />);

    expect(screen.getByText('Never Get Stuck')).toBeInTheDocument();
  });

  it('renders the AI Tutor feature description', () => {
    render(<FeaturesSection />);

    expect(
      screen.getByText(/AI-powered tutor understands robotics/i)
    ).toBeInTheDocument();
  });

  it('renders the Personalization feature title', () => {
    render(<FeaturesSection />);

    expect(screen.getByText('Hardware or Software?')).toBeInTheDocument();
  });

  it('renders the Personalization feature description', () => {
    render(<FeaturesSection />);

    expect(
      screen.getByText(/Toggle between code-focused and hardware-focused/i)
    ).toBeInTheDocument();
  });

  it('renders the ChatbotDemo component', () => {
    render(<FeaturesSection />);

    // ChatbotDemo renders AI Tutor header
    expect(screen.getByText('AI Tutor')).toBeInTheDocument();
  });

  it('renders the PersonalizationDemo component', () => {
    render(<FeaturesSection />);

    // PersonalizationDemo renders Learning Path header
    expect(screen.getByText('Learning Path')).toBeInTheDocument();
  });
});
