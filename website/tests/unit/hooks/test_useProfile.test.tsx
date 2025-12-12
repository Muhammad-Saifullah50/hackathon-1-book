// website/tests/unit/hooks/test_useProfile.test.tsx
import { renderHook, act, waitFor } from '@testing-library/react';
import { ProfileProvider, useProfile } from '@/hooks/useProfile';
import { AuthProvider } from '@/hooks/useAuth';
import { useAuth } from '@/hooks/useAuth';
import { enableFetchMocks } from 'jest-fetch-mock';
import { v4 as uuidv4 } from 'uuid';
import React from 'react';

enableFetchMocks();

// Mock the useAuth hook to control user state
jest.mock('@/hooks/useAuth', () => ({
  useAuth: jest.fn(),
  AuthProvider: ({ children }: { children: React.ReactNode }) => <div>{children}</div>
}));

const mockUser = { id: uuidv4(), email: 'user@example.com' };

const wrapper = ({ children }: { children: React.ReactNode }) => (
  <ProfileProvider>{children}</ProfileProvider>
);

describe('useProfile', () => {
  beforeEach(() => {
    fetchMock.resetMocks();
    (useAuth as jest.Mock).mockReturnValue({
      user: mockUser,
      loading: false,
      login: jest.fn(),
      signup: jest.fn(),
      logout: jest.fn(),
    });
  });

  it('should initialize with loading state and no profile', async () => {
    fetchMock.mockResponseOnce(JSON.stringify({ detail: 'Profile not found' }), { status: 404 });

    const { result } = renderHook(() => useProfile(), { wrapper });

    expect(result.current.loadingProfile).toBe(true);
    expect(result.current.profile).toBe(null);

    await waitFor(() => expect(result.current.loadingProfile).toBe(false));
    expect(result.current.profile).toBe(null);
  });

  it('should fetch profile on initial load if user is authenticated and profile exists', async () => {
    const mockProfile = { user_id: mockUser.id, age_range: '25_34', preferred_language: 'en' };
    fetchMock.mockResponseOnce(JSON.stringify(mockProfile), { status: 200 });

    const { result } = renderHook(() => useProfile(), { wrapper });

    await waitFor(() => expect(result.current.loadingProfile).toBe(false));
    expect(result.current.profile).toEqual(mockProfile);
  });

  it('should create a profile successfully', async () => {
    // Mock initial fetch profile as 404
    // Then mock create API call as 201
    const newProfileData = { age_range: '18_24' as any, time_per_week: 10 };
    const createdProfile = { ...newProfileData, user_id: mockUser.id, preferred_language: 'en' };
    
    fetchMock
        .mockResponseOnce(JSON.stringify({ detail: 'Profile not found' }), { status: 404 })
        .mockResponseOnce(JSON.stringify(createdProfile), { status: 201 });

    const { result } = renderHook(() => useProfile(), { wrapper });
    await waitFor(() => expect(result.current.loadingProfile).toBe(false)); // Wait for initial fetch to complete

    await act(async () => {
      await result.current.createOrUpdateProfile(newProfileData);
    });

    await waitFor(() => expect(result.current.profile).toEqual(createdProfile));
    expect(result.current.loadingProfile).toBe(false);
  });

  it('should update an existing profile successfully', async () => {
    const existingProfile = { user_id: mockUser.id, age_range: '18_24', preferred_language: 'en' };
    
    const updateData = { time_per_week: 20, learning_mode: 'visual' as any };
    const updatedProfile = { ...existingProfile, ...updateData };

    fetchMock
        .mockResponseOnce(JSON.stringify(existingProfile), { status: 200 })
        .mockResponseOnce(JSON.stringify(updatedProfile), { status: 200 });

    const { result } = renderHook(() => useProfile(), { wrapper });
    await waitFor(() => expect(result.current.loadingProfile).toBe(false));
    expect(result.current.profile).toEqual(existingProfile);

    await act(async () => {
      await result.current.createOrUpdateProfile(updateData);
    });

    expect(result.current.profile).toEqual(updatedProfile);
    expect(result.current.loadingProfile).toBe(false);
  });

  it('should clear profile if user logs out', async () => {
    // Initial state: user logged in
    const mockProfile = { user_id: mockUser.id, age_range: '25_34', preferred_language: 'en' };
    fetchMock.mockResponseOnce(JSON.stringify(mockProfile), { status: 200 });

    const { result, rerender } = renderHook(() => useProfile(), { wrapper });
    await waitFor(() => expect(result.current.loadingProfile).toBe(false));
    expect(result.current.profile).toEqual(mockProfile);

    // Simulate logout
    (useAuth as jest.Mock).mockReturnValue({
      user: null, // Simulate user logged out
      loading: false,
      login: jest.fn(),
      signup: jest.fn(),
      logout: jest.fn(),
    });

    // Re-render to trigger effect with new auth state
    rerender();

    await waitFor(() => expect(result.current.profile).toBe(null));
  });
});