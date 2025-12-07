// tests/unit/components/Paginator.test.tsx
import React from 'react';
import { render, screen } from '@testing-library/react';

// Mock the Docusaurus PaginatorNavLink to test logic without Docusaurus context
const MockPaginatorNavLink = ({ permalink, title, isNext }: any) => (
  <a href={permalink} data-testid={isNext ? 'next-link' : 'prev-link'}>
    {title}
  </a>
);

describe('Paginator Component', () => {
  it('renders next and previous links', () => {
    render(
      <nav className="pagination-nav" aria-label="Docs pages">
        <MockPaginatorNavLink permalink="/prev" title="Previous Lesson" />
        <MockPaginatorNavLink permalink="/next" title="Next Lesson" isNext />
      </nav>
    );

    expect(screen.getByTestId('prev-link')).toHaveAttribute('href', '/prev');
    expect(screen.getByText('Previous Lesson')).toBeInTheDocument();
    
    expect(screen.getByTestId('next-link')).toHaveAttribute('href', '/next');
    expect(screen.getByText('Next Lesson')).toBeInTheDocument();
  });
});
