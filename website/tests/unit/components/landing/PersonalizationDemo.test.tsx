import { render, screen, fireEvent } from '@testing-library/react';
import { PersonalizationDemo } from '@/components/landing/PersonalizationDemo';

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

describe('PersonalizationDemo', () => {
  it('renders the Learning Path header', () => {
    render(<PersonalizationDemo />);

    expect(screen.getByText('Learning Path')).toBeInTheDocument();
  });

  it('starts in code focus mode', () => {
    render(<PersonalizationDemo />);

    expect(screen.getByText('Software Focus')).toBeInTheDocument();
  });

  it('renders the toggle button', () => {
    render(<PersonalizationDemo />);

    const toggleButton = screen.getByRole('button', { name: /Switch/i });
    expect(toggleButton).toBeInTheDocument();
  });

  it('shows Python code snippet in code mode', () => {
    render(<PersonalizationDemo />);

    expect(screen.getByText(/# Python ROS 2 Node/i)).toBeInTheDocument();
    expect(screen.getByText('rclpy')).toBeInTheDocument();
  });

  it('shows code tags in code mode', () => {
    render(<PersonalizationDemo />);

    expect(screen.getByText('Python')).toBeInTheDocument();
    expect(screen.getByText('ROS 2')).toBeInTheDocument();
    expect(screen.getByText('Simulation')).toBeInTheDocument();
  });

  it('toggles to hardware mode when switch is clicked', () => {
    render(<PersonalizationDemo />);

    const toggleButton = screen.getByRole('button', { name: /Switch to hardware focus/i });
    fireEvent.click(toggleButton);

    expect(screen.getByText('Hardware Focus')).toBeInTheDocument();
  });

  it('shows hardware tags after toggling to hardware mode', () => {
    render(<PersonalizationDemo />);

    const toggleButton = screen.getByRole('button', { name: /Switch/i });
    fireEvent.click(toggleButton);

    expect(screen.getByText('Hardware')).toBeInTheDocument();
    expect(screen.getByText('Jetson')).toBeInTheDocument();
    expect(screen.getByText('Physical')).toBeInTheDocument();
  });

  it('renders footer text about content adaptation', () => {
    render(<PersonalizationDemo />);

    expect(
      screen.getByText(/Content adapts to your learning preference/i)
    ).toBeInTheDocument();
  });
});
