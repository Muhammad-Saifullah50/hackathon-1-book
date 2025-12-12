import { render, screen } from '@testing-library/react';
import { HardwareItem } from '@/components/landing/HardwareItem';
import { Cpu } from 'lucide-react';

describe('HardwareItem', () => {
  const defaultProps = {
    icon: Cpu,
    category: 'Brain',
    name: 'NVIDIA Jetson Orin',
    description: 'Edge AI computing platform',
  };

  it('renders the category', () => {
    render(<HardwareItem {...defaultProps} />);

    expect(screen.getByText('Brain')).toBeInTheDocument();
  });

  it('renders the hardware name', () => {
    render(<HardwareItem {...defaultProps} />);

    expect(screen.getByText('NVIDIA Jetson Orin')).toBeInTheDocument();
  });

  it('renders the description', () => {
    render(<HardwareItem {...defaultProps} />);

    expect(screen.getByText('Edge AI computing platform')).toBeInTheDocument();
  });

  it('shows Required status by default', () => {
    render(<HardwareItem {...defaultProps} />);

    expect(screen.getByText('Required')).toBeInTheDocument();
  });

  it('shows OPTIONAL label when isOptional is true', () => {
    render(<HardwareItem {...defaultProps} isOptional />);

    expect(screen.getByText('OPTIONAL')).toBeInTheDocument();
  });

  it('shows cloud alternative status when optional', () => {
    render(<HardwareItem {...defaultProps} isOptional />);

    expect(screen.getByText('Cloud alternative available')).toBeInTheDocument();
  });

  it('applies custom className', () => {
    const { container } = render(
      <HardwareItem {...defaultProps} className="custom-class" />
    );

    expect(container.firstChild).toHaveClass('custom-class');
  });

  it('has game-style tech aesthetic', () => {
    const { container } = render(<HardwareItem {...defaultProps} />);

    const card = container.firstChild;
    expect(card).toHaveClass('bg-slate-900/60');
    expect(card).toHaveClass('backdrop-blur-sm');
  });
});
