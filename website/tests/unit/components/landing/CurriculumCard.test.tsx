import { render, screen } from '@testing-library/react';
import { CurriculumCard } from '@/components/landing/CurriculumCard';
import { Network } from 'lucide-react';

describe('CurriculumCard', () => {
  const defaultProps = {
    icon: Network,
    title: 'The Nervous System',
    subtitle: 'ROS 2 Middleware & Nodes',
    weekRange: 'Weeks 1-5',
  };

  it('renders the title', () => {
    render(<CurriculumCard {...defaultProps} />);

    expect(screen.getByText('The Nervous System')).toBeInTheDocument();
  });

  it('renders the subtitle', () => {
    render(<CurriculumCard {...defaultProps} />);

    expect(screen.getByText('ROS 2 Middleware & Nodes')).toBeInTheDocument();
  });

  it('renders the week range', () => {
    render(<CurriculumCard {...defaultProps} />);

    expect(screen.getByText('Weeks 1-5')).toBeInTheDocument();
  });

  it('applies glassmorphism styling', () => {
    const { container } = render(<CurriculumCard {...defaultProps} />);

    const card = container.firstChild;
    expect(card).toHaveClass('bg-slate-900/50');
    expect(card).toHaveClass('backdrop-blur-md');
    expect(card).toHaveClass('border');
  });

  it('applies custom className', () => {
    const { container } = render(
      <CurriculumCard {...defaultProps} className="custom-class" />
    );

    expect(container.firstChild).toHaveClass('custom-class');
  });

  it('has hover effect classes', () => {
    const { container } = render(<CurriculumCard {...defaultProps} />);

    const card = container.firstChild;
    expect(card).toHaveClass('hover:-translate-y-1');
    expect(card).toHaveClass('hover:bg-slate-800/60');
  });

  it('has accessibility focus styles', () => {
    const { container } = render(<CurriculumCard {...defaultProps} />);

    const card = container.firstChild;
    expect(card).toHaveClass('focus-within:ring-2');
  });
});
