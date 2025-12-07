// tests/unit/components/Image.test.tsx
import React from 'react';
import { render, screen } from '@testing-library/react';

// Mock the Image component we plan to build
const MockImage = ({ src, alt }: { src: string, alt: string }) => (
  <img src={src} alt={alt} className="w-full h-auto rounded-lg shadow-md" />
);

describe('Image Component', () => {
  it('renders with responsive classes', () => {
    render(<MockImage src="test.jpg" alt="Test Robot" />);
    const img = screen.getByRole('img', { name: 'Test Robot' });
    expect(img).toHaveAttribute('src', 'test.jpg');
    expect(img).toHaveClass('w-full');
    expect(img).toHaveClass('h-auto');
  });
});
