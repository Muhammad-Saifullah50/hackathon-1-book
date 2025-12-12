// website/tests/unit/components/profile/PersonalizationBar.test.tsx
import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import PersonalizationBar from '@/components/profile/PersonalizationBar'; // Updated import
import { useProfile } from '@/hooks/useProfile'; // Updated import

// Mock the useProfile hook
jest.mock('@/hooks/useProfile', () => ({
  useProfile: jest.fn(),
}));

describe('PersonalizationBar', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  it('does not render when loading', () => {
    (useProfile as jest.Mock).mockReturnValue({
      profile: null,
      loadingProfile: true,
    });
    const { container } = render(<PersonalizationBar />);
    expect(container).toBeEmptyDOMElement();
  });

  it('does not render when no profile exists', () => {
    (useProfile as jest.Mock).mockReturnValue({
      profile: null,
      loadingProfile: false,
    });
    const { container } = render(<PersonalizationBar />);
    expect(container).toBeEmptyDOMElement();
  });

  it('renders when profile exists', () => {
    (useProfile as jest.Mock).mockReturnValue({
      profile: {
        user_id: '123',
        tech_background: 'software_engineer',
      },
      loadingProfile: false,
    });
    render(<PersonalizationBar />);
    expect(screen.getByText(/Personalize this page?/i)).toBeInTheDocument();
    expect(screen.getByText(/software engineer/i)).toBeInTheDocument();
  });

  it('triggers logic on button click', () => {
    (useProfile as jest.Mock).mockReturnValue({
      profile: {
        user_id: '123',
        tech_background: 'student',
        time_per_week: 10,
      },
      loadingProfile: false,
    });
    
    // Mock console.log and alert
    const consoleSpy = jest.spyOn(console, 'log').mockImplementation();
    const alertSpy = jest.spyOn(window, 'alert').mockImplementation();

    render(<PersonalizationBar />);
    
    fireEvent.click(screen.getByText('Personalize Page'));

    expect(consoleSpy).toHaveBeenCalledWith('Personalize Page clicked for user:', '123');
    expect(alertSpy).toHaveBeenCalledWith(expect.stringContaining('Personalization logic triggered'));

    consoleSpy.mockRestore();
    alertSpy.mockRestore();
  });
});