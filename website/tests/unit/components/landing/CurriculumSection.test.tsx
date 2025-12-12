import { render, screen } from '@testing-library/react';
import { CurriculumSection } from '@/components/landing/CurriculumSection';

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

describe('CurriculumSection', () => {
  it('renders the section title', () => {
    render(<CurriculumSection />);

    expect(screen.getByText(/The Journey to/i)).toBeInTheDocument();
    expect(screen.getByText(/Embodied Intelligence/i)).toBeInTheDocument();
  });

  it('renders all four curriculum cards', () => {
    render(<CurriculumSection />);

    expect(screen.getByText('The Nervous System')).toBeInTheDocument();
    expect(screen.getByText('The Digital Twin')).toBeInTheDocument();
    expect(screen.getByText('The Brain')).toBeInTheDocument();
    expect(screen.getByText('The Body')).toBeInTheDocument();
  });

  it('renders correct subtitles for each card', () => {
    render(<CurriculumSection />);

    expect(screen.getByText('ROS 2 Middleware & Nodes')).toBeInTheDocument();
    expect(screen.getByText('Simulation in Gazebo & Unity')).toBeInTheDocument();
    expect(screen.getByText('NVIDIA Isaac & Perception')).toBeInTheDocument();
    expect(screen.getByText('VLA & Humanoid Control')).toBeInTheDocument();
  });

  it('renders week ranges for each card', () => {
    render(<CurriculumSection />);

    expect(screen.getByText('Weeks 1-5')).toBeInTheDocument();
    expect(screen.getByText('Weeks 6-7')).toBeInTheDocument();
    expect(screen.getByText('Weeks 8-10')).toBeInTheDocument();
    expect(screen.getByText('Weeks 11-13')).toBeInTheDocument();
  });

  it('renders the curriculum overview badge', () => {
    render(<CurriculumSection />);

    expect(screen.getByText('Curriculum Overview')).toBeInTheDocument();
  });

  it('has the curriculum section ID for anchor linking', () => {
    const { container } = render(<CurriculumSection />);

    const section = container.querySelector('#curriculum');
    expect(section).toBeInTheDocument();
  });

  it('has responsive grid layout classes', () => {
    render(<CurriculumSection />);

    // Find the grid container by checking for cards
    const cards = screen.getAllByText(/Weeks/);
    const grid = cards[0].closest('.grid');
    expect(grid).toHaveClass('grid-cols-1');
    expect(grid).toHaveClass('lg:grid-cols-4');
  });
});
