import { render, screen } from '@testing-library/react';
import { GlowButton } from '@/components/landing/GlowButton';

describe('GlowButton', () => {
  it('renders with children text', () => {
    render(<GlowButton href="/test">Click me</GlowButton>);

    expect(screen.getByText('Click me')).toBeInTheDocument();
  });

  it('renders as internal link by default', () => {
    render(<GlowButton href="/docs">Internal Link</GlowButton>);

    const link = screen.getByRole('link', { name: 'Internal Link' });
    expect(link).toHaveAttribute('href', '/docs');
    expect(link).not.toHaveAttribute('target');
  });

  it('renders as external link when external prop is true', () => {
    render(
      <GlowButton href="https://example.com" external>
        External Link
      </GlowButton>
    );

    const link = screen.getByRole('link', { name: 'External Link' });
    expect(link).toHaveAttribute('href', 'https://example.com');
    expect(link).toHaveAttribute('target', '_blank');
    expect(link).toHaveAttribute('rel', 'noopener noreferrer');
  });

  it('applies primary variant styles by default', () => {
    render(<GlowButton href="/test">Primary Button</GlowButton>);

    const button = screen.getByRole('link', { name: 'Primary Button' });
    expect(button).toHaveClass('bg-cyan-400');
    expect(button).toHaveClass('text-slate-950');
  });

  it('applies secondary variant styles when specified', () => {
    render(
      <GlowButton href="/test" variant="secondary">
        Secondary Button
      </GlowButton>
    );

    const button = screen.getByRole('link', { name: 'Secondary Button' });
    expect(button).toHaveClass('bg-transparent');
    expect(button).toHaveClass('border-white');
  });

  it('applies custom className', () => {
    render(
      <GlowButton href="/test" className="custom-class">
        Custom Button
      </GlowButton>
    );

    const button = screen.getByRole('link', { name: 'Custom Button' });
    expect(button).toHaveClass('custom-class');
  });

  it('has proper focus styles for accessibility', () => {
    render(<GlowButton href="/test">Accessible Button</GlowButton>);

    const button = screen.getByRole('link', { name: 'Accessible Button' });
    expect(button).toHaveClass('focus-visible:ring-2');
  });
});
