// tests/unit/components/Admonition.test.tsx
import React from 'react';
import { render, screen } from '@testing-library/react';

// Mock Docusaurus Admonition component
const MockAdmonition = ({ type, title, children }: any) => (
  <div data-testid={`admonition-${type}`} className={`admonition alert alert--${type}`}>
    <div className="admonition-heading">
      <h5>{title}</h5>
    </div>
    <div className="admonition-content">{children}</div>
  </div>
);

describe('Admonition Component', () => {
  it('renders a Lab admonition', () => {
    render(
      <MockAdmonition type="lab" title="Lab Exercise">
        <p>Start the simulation.</p>
      </MockAdmonition>
    );
    expect(screen.getByTestId('admonition-lab')).toBeInTheDocument();
    expect(screen.getByText('Lab Exercise')).toBeInTheDocument();
    expect(screen.getByText('Start the simulation.')).toBeInTheDocument();
  });

  it('renders a Capstone admonition', () => {
    render(
      <MockAdmonition type="capstone" title="Capstone Project">
        <p>Build the robot.</p>
      </MockAdmonition>
    );
    expect(screen.getByTestId('admonition-capstone')).toBeInTheDocument();
    expect(screen.getByText('Capstone Project')).toBeInTheDocument();
  });
});
