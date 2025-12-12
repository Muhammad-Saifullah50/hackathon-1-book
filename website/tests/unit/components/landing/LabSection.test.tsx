import { render, screen } from '@testing-library/react';
import { LabSection } from '@/components/landing/LabSection';

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

describe('LabSection', () => {
  it('renders the section title', () => {
    render(<LabSection />);

    expect(screen.getByText(/Required/i)).toBeInTheDocument();
    expect(screen.getByText(/Equipment/i)).toBeInTheDocument();
  });

  it('renders the Hardware Lab badge', () => {
    render(<LabSection />);

    expect(screen.getByText('Hardware Lab')).toBeInTheDocument();
  });

  it('renders the Brain hardware item', () => {
    render(<LabSection />);

    expect(screen.getByText('Brain')).toBeInTheDocument();
    expect(screen.getByText('NVIDIA Jetson Orin')).toBeInTheDocument();
  });

  it('renders the Eyes hardware item', () => {
    render(<LabSection />);

    expect(screen.getByText('Eyes')).toBeInTheDocument();
    expect(screen.getByText('Intel RealSense D435')).toBeInTheDocument();
  });

  it('renders the Body hardware item', () => {
    render(<LabSection />);

    expect(screen.getByText('Body')).toBeInTheDocument();
    expect(screen.getByText('Unitree Go2 / G1')).toBeInTheDocument();
  });

  it('shows Body item as optional', () => {
    render(<LabSection />);

    expect(screen.getByText('OPTIONAL')).toBeInTheDocument();
  });

  it('renders the cloud simulation alternative', () => {
    render(<LabSection />);

    expect(screen.getByText('Cloud Simulation Available')).toBeInTheDocument();
    expect(
      screen.getByText(/Run everything in our cloud-based/i)
    ).toBeInTheDocument();
  });

  it('renders the loadout configuration header', () => {
    render(<LabSection />);

    expect(screen.getByText('Loadout Configuration')).toBeInTheDocument();
  });
});
