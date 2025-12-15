// website/tests/unit/components/profile/PersonalizationBar.test.tsx
/**
 * Unit tests for PersonalizationBar component.
 *
 * Tests:
 * - Authentication states (logged out, logged in without profile, logged in with profile)
 * - Loading states during personalization
 * - Error and quota exceeded states
 * - Personalized state with toggle functionality
 * - Cached version availability
 * - Re-personalization for stale profiles
 */

import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import PersonalizationBar from '@/components/profile/PersonalizationBar';

// Mock all dependencies
jest.mock('@docusaurus/router', () => ({
  useLocation: jest.fn(() => ({ pathname: '/docs/intro' })),
}));

jest.mock('@/hooks/useProfile.tsx', () => ({
  useProfile: jest.fn(),
}));

jest.mock('@/hooks/useAuth', () => ({
  useAuth: jest.fn(),
}));

jest.mock('@/hooks/useSafeColorMode', () => ({
  useSafeColorMode: jest.fn(() => ({ isDark: false })),
}));

jest.mock('@/hooks/usePersonalization', () => ({
  usePersonalization: jest.fn(),
}));

import { useProfile } from '@/hooks/useProfile.tsx';
import { useAuth } from '@/hooks/useAuth';
import { usePersonalization } from '@/hooks/usePersonalization';

// Default mock implementations
const mockPersonalizeFunction = jest.fn();
const mockToggleView = jest.fn();
const mockLoadCachedVersion = jest.fn();

const defaultPersonalizationState = {
  state: 'idle',
  viewMode: 'original',
  error: null,
  quotaStatus: { limit: 5, used: 0, remaining: 5, resetsAt: new Date().toISOString() },
  personalizedContent: null,
  hasCachedVersion: false,
  isProfileStale: false,
  personalize: mockPersonalizeFunction,
  toggleView: mockToggleView,
  loadCachedVersion: mockLoadCachedVersion,
};

const mockProfile = {
  user_id: 'user123',
  tech_background: 'intermediate',
  learning_mode: 'visual',
};

const mockUser = {
  id: 'user123',
  email: 'test@example.com',
};

beforeEach(() => {
  jest.clearAllMocks();

  // Default: logged in with profile
  (useAuth as jest.Mock).mockReturnValue({
    user: mockUser,
    loading: false,
  });

  (useProfile as jest.Mock).mockReturnValue({
    profile: mockProfile,
    loadingProfile: false,
  });

  (usePersonalization as jest.Mock).mockReturnValue(defaultPersonalizationState);
});

describe('PersonalizationBar', () => {
  describe('Authentication States', () => {
    it('does not render when loading auth', () => {
      (useAuth as jest.Mock).mockReturnValue({
        user: null,
        loading: true,
      });

      const { container } = render(<PersonalizationBar />);
      expect(container).toBeEmptyDOMElement();
    });

    it('does not render when loading profile', () => {
      (useProfile as jest.Mock).mockReturnValue({
        profile: null,
        loadingProfile: true,
      });

      const { container } = render(<PersonalizationBar />);
      expect(container).toBeEmptyDOMElement();
    });

    it('shows login prompt when not logged in', () => {
      (useAuth as jest.Mock).mockReturnValue({
        user: null,
        loading: false,
      });

      render(<PersonalizationBar />);

      expect(screen.getByText(/Personalize this page/i)).toBeInTheDocument();
      expect(screen.getByText(/Log in to tailor content/i)).toBeInTheDocument();
      expect(screen.getByRole('link', { name: /Log in/i })).toHaveAttribute('href', '/login');
    });

    it('shows complete profile prompt when logged in without profile', () => {
      (useProfile as jest.Mock).mockReturnValue({
        profile: null,
        loadingProfile: false,
      });

      render(<PersonalizationBar />);

      expect(screen.getByText(/Complete your profile/i)).toBeInTheDocument();
      expect(screen.getByText(/Set up your learning preferences/i)).toBeInTheDocument();
      expect(screen.getByRole('link', { name: /Complete Profile/i })).toHaveAttribute(
        'href',
        '/signup-wizard'
      );
    });

    it('shows personalize option when logged in with profile', () => {
      render(<PersonalizationBar />);

      expect(screen.getByText(/Personalize this page\?/i)).toBeInTheDocument();
      expect(screen.getByText(/intermediate/i)).toBeInTheDocument();
      expect(screen.getByRole('button', { name: /Personalize Page/i })).toBeInTheDocument();
    });
  });

  describe('Loading States (T035)', () => {
    it('shows loading state during personalization', () => {
      (usePersonalization as jest.Mock).mockReturnValue({
        ...defaultPersonalizationState,
        state: 'loading',
      });

      render(<PersonalizationBar />);

      expect(screen.getByText(/Personalizing content\.\.\./i)).toBeInTheDocument();
      expect(screen.getByText(/Adapting content/i)).toBeInTheDocument();
      expect(screen.getByText(/Processing\.\.\./i)).toBeInTheDocument();
    });

    it('shows spinner animation during loading', () => {
      (usePersonalization as jest.Mock).mockReturnValue({
        ...defaultPersonalizationState,
        state: 'loading',
      });

      const { container } = render(<PersonalizationBar />);

      // Check for animate-spin class on the loader icon
      const spinnerIcon = container.querySelector('.animate-spin');
      expect(spinnerIcon).toBeInTheDocument();
    });

    it('displays profile tech background in loading message', () => {
      (usePersonalization as jest.Mock).mockReturnValue({
        ...defaultPersonalizationState,
        state: 'loading',
      });

      render(<PersonalizationBar />);

      expect(screen.getByText(/intermediate/i)).toBeInTheDocument();
    });
  });

  describe('Error States', () => {
    it('shows error state with retry button', () => {
      (usePersonalization as jest.Mock).mockReturnValue({
        ...defaultPersonalizationState,
        state: 'error',
        error: 'Something went wrong. Please try again.',
      });

      render(<PersonalizationBar />);

      expect(screen.getByText(/Personalization failed/i)).toBeInTheDocument();
      expect(screen.getByText(/Something went wrong/i)).toBeInTheDocument();
      expect(screen.getByRole('button', { name: /Retry/i })).toBeInTheDocument();
    });

    it('shows quota exceeded state without retry', () => {
      (usePersonalization as jest.Mock).mockReturnValue({
        ...defaultPersonalizationState,
        state: 'quota_exceeded',
        error: 'Daily personalization limit reached. Try again tomorrow!',
      });

      render(<PersonalizationBar />);

      expect(screen.getByText(/Daily limit reached/i)).toBeInTheDocument();
      expect(screen.getByText(/limit reached/i)).toBeInTheDocument();
      // Retry button should NOT be present for quota exceeded
      expect(screen.queryByRole('button', { name: /Retry/i })).not.toBeInTheDocument();
    });

    it('calls personalize on retry button click', () => {
      (usePersonalization as jest.Mock).mockReturnValue({
        ...defaultPersonalizationState,
        state: 'error',
        error: 'Something went wrong',
      });

      render(<PersonalizationBar />);

      fireEvent.click(screen.getByRole('button', { name: /Retry/i }));

      expect(mockPersonalizeFunction).toHaveBeenCalled();
    });
  });

  describe('Personalized State', () => {
    it('shows toggle button when personalized', () => {
      (usePersonalization as jest.Mock).mockReturnValue({
        ...defaultPersonalizationState,
        state: 'personalized',
        viewMode: 'personalized',
        personalizedContent: 'Personalized content here',
      });

      render(<PersonalizationBar />);

      expect(screen.getByText(/Viewing personalized content/i)).toBeInTheDocument();
      expect(screen.getByRole('button', { name: /Show Original/i })).toBeInTheDocument();
    });

    it('shows original view message when toggled', () => {
      (usePersonalization as jest.Mock).mockReturnValue({
        ...defaultPersonalizationState,
        state: 'personalized',
        viewMode: 'original',
        personalizedContent: 'Personalized content here',
      });

      render(<PersonalizationBar />);

      expect(screen.getByText(/Viewing original content/i)).toBeInTheDocument();
      expect(screen.getByRole('button', { name: /Show Personalized/i })).toBeInTheDocument();
    });

    it('calls toggleView on button click', () => {
      (usePersonalization as jest.Mock).mockReturnValue({
        ...defaultPersonalizationState,
        state: 'personalized',
        viewMode: 'personalized',
        personalizedContent: 'Personalized content here',
      });

      render(<PersonalizationBar />);

      fireEvent.click(screen.getByRole('button', { name: /Show Original/i }));

      expect(mockToggleView).toHaveBeenCalled();
    });

    it('displays remaining quota in personalized state', () => {
      (usePersonalization as jest.Mock).mockReturnValue({
        ...defaultPersonalizationState,
        state: 'personalized',
        viewMode: 'personalized',
        quotaStatus: { limit: 5, used: 3, remaining: 2, resetsAt: new Date().toISOString() },
      });

      render(<PersonalizationBar />);

      expect(screen.getByText(/2\/5 personalizations left today/i)).toBeInTheDocument();
    });
  });

  describe('Cached Version Handling', () => {
    it('shows view personalized option when cache exists', () => {
      (usePersonalization as jest.Mock).mockReturnValue({
        ...defaultPersonalizationState,
        hasCachedVersion: true,
      });

      render(<PersonalizationBar />);

      expect(screen.getByText(/Personalized version available/i)).toBeInTheDocument();
      expect(screen.getByRole('button', { name: /View Personalized/i })).toBeInTheDocument();
    });

    it('calls loadCachedVersion on view button click', () => {
      (usePersonalization as jest.Mock).mockReturnValue({
        ...defaultPersonalizationState,
        hasCachedVersion: true,
      });

      render(<PersonalizationBar />);

      fireEvent.click(screen.getByRole('button', { name: /View Personalized/i }));

      expect(mockLoadCachedVersion).toHaveBeenCalled();
    });
  });

  describe('Stale Profile Handling', () => {
    it('shows re-personalize option when profile is stale and cached', () => {
      (usePersonalization as jest.Mock).mockReturnValue({
        ...defaultPersonalizationState,
        hasCachedVersion: true,
        isProfileStale: true,
      });

      render(<PersonalizationBar />);

      expect(screen.getByText(/Your profile has changed/i)).toBeInTheDocument();
      expect(screen.getByRole('button', { name: /Re-personalize \(Free\)/i })).toBeInTheDocument();
    });

    it('shows re-personalize button in personalized state when stale', () => {
      (usePersonalization as jest.Mock).mockReturnValue({
        ...defaultPersonalizationState,
        state: 'personalized',
        viewMode: 'personalized',
        isProfileStale: true,
      });

      render(<PersonalizationBar />);

      expect(screen.getByRole('button', { name: /Re-personalize/i })).toBeInTheDocument();
    });

    it('calls personalize when re-personalize is clicked', () => {
      (usePersonalization as jest.Mock).mockReturnValue({
        ...defaultPersonalizationState,
        hasCachedVersion: true,
        isProfileStale: true,
      });

      render(<PersonalizationBar />);

      fireEvent.click(screen.getByRole('button', { name: /Re-personalize \(Free\)/i }));

      expect(mockPersonalizeFunction).toHaveBeenCalled();
    });
  });

  describe('Quota Display', () => {
    it('displays quota in idle state', () => {
      (usePersonalization as jest.Mock).mockReturnValue({
        ...defaultPersonalizationState,
        quotaStatus: { limit: 5, used: 2, remaining: 3, resetsAt: new Date().toISOString() },
      });

      render(<PersonalizationBar />);

      expect(screen.getByText(/3\/5 left today/i)).toBeInTheDocument();
    });

    it('disables personalize button when quota is 0', () => {
      (usePersonalization as jest.Mock).mockReturnValue({
        ...defaultPersonalizationState,
        quotaStatus: { limit: 5, used: 5, remaining: 0, resetsAt: new Date().toISOString() },
      });

      render(<PersonalizationBar />);

      const button = screen.getByRole('button', { name: /Personalize Page/i });
      expect(button).toBeDisabled();
    });
  });

  describe('Button Click Handlers', () => {
    it('calls personalize on Personalize Page button click', () => {
      render(<PersonalizationBar />);

      fireEvent.click(screen.getByRole('button', { name: /Personalize Page/i }));

      expect(mockPersonalizeFunction).toHaveBeenCalled();
    });
  });

  describe('Dark Mode Support', () => {
    it('applies dark mode styles', () => {
      jest.requireMock('@/hooks/useSafeColorMode').useSafeColorMode.mockReturnValue({
        isDark: true,
      });

      const { container } = render(<PersonalizationBar />);

      // Check that dark mode class variants are applied
      const mainContainer = container.firstChild;
      expect(mainContainer).toHaveClass('bg-gradient-to-r');
    });
  });
});
