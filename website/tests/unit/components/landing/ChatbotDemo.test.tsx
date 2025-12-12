import { render, screen } from '@testing-library/react';
import { ChatbotDemo } from '@/components/landing/ChatbotDemo';

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
  useReducedMotion: jest.fn(() => true), // Reduced motion to show all messages immediately
}));

describe('ChatbotDemo', () => {
  it('renders the AI Tutor header', () => {
    render(<ChatbotDemo />);

    expect(screen.getByText('AI Tutor')).toBeInTheDocument();
    expect(screen.getByText('Always here to help')).toBeInTheDocument();
  });

  it('renders online status indicator', () => {
    render(<ChatbotDemo />);

    expect(screen.getByText('Online')).toBeInTheDocument();
  });

  it('renders the user question about quaternions', () => {
    render(<ChatbotDemo />);

    expect(
      screen.getByText(/What are quaternions/i)
    ).toBeInTheDocument();
  });

  it('renders the assistant response about quaternions', () => {
    render(<ChatbotDemo />);

    expect(
      screen.getByText(/mathematical representation for 3D rotations/i)
    ).toBeInTheDocument();
  });

  it('renders the chat input placeholder', () => {
    render(<ChatbotDemo />);

    expect(screen.getByText('Ask anything...')).toBeInTheDocument();
  });

  it('has glassmorphism styling on chat window', () => {
    const { container } = render(<ChatbotDemo />);

    const chatWindow = container.querySelector('.backdrop-blur-md');
    expect(chatWindow).toBeInTheDocument();
  });
});
