// website/tests/unit/hooks/test_useAuth.test.tsx

import { renderHook, act, waitFor } from '@testing-library/react';
import { useAuth, AuthProvider } from '../../src/hooks/useAuth.tsx'; // Updated import
import { enableFetchMocks } from 'jest-fetch-mock';

enableFetchMocks();

describe('useAuth', () => {
  beforeEach(() => {
    fetchMock.resetMocks();
  });

  it('should initialize with loading state and no user', async () => {
    fetchMock.mockResponseOnce(JSON.stringify({}), { status: 401 }); // Simulate no user authenticated
    const { result } = renderHook(() => useAuth(), { wrapper: AuthProvider });

    expect(result.current.loading).toBe(true);
    expect(result.current.user).toBe(null);

    await waitFor(() => expect(result.current.loading).toBe(false));
    expect(result.current.user).toBe(null);
  });

  it('should handle successful login', async () => {
    const mockUser = { id: '123', email: 'test@example.com' };
    fetchMock.mockResponses(
      [JSON.stringify({}), { status: 401 }], // Initial /api/auth/me call
      [JSON.stringify(mockUser), { status: 200 }] // /api/auth/login call
    );

    const { result } = renderHook(() => useAuth(), { wrapper: AuthProvider });
    await waitFor(() => expect(result.current.loading).toBe(false));

    await act(async () => {
      await result.current.login({ email: 'test@example.com', password: 'password123' });
    });

    expect(result.current.user).toEqual(mockUser);
    expect(result.current.loading).toBe(false);
  });

  it('should handle successful signup', async () => {
    const mockUser = { id: '456', email: 'new@example.com' };
    fetchMock.mockResponses(
      [JSON.stringify({}), { status: 401 }], // Initial /api/auth/me call
      [JSON.stringify(mockUser), { status: 201 }] // /api/auth/signup call
    );

    const { result } = renderHook(() => useAuth(), { wrapper: AuthProvider });
    await waitFor(() => expect(result.current.loading).toBe(false));

    await act(async () => {
      await result.current.signup({ email: 'new@example.com', password: 'newpassword' });
    });

    expect(result.current.user).toEqual(mockUser);
    expect(result.current.loading).toBe(false);
  });

  it('should handle successful logout', async () => {
    const mockUser = { id: '123', email: 'test@example.com' };
    fetchMock.mockResponses(
      [JSON.stringify(mockUser), { status: 200 }], // Initial /api/auth/me call
      [JSON.stringify({ message: 'Logout successful' }), { status: 200 }] // /api/auth/logout call
    );

    const { result } = renderHook(() => useAuth(), { wrapper: AuthProvider });
    await waitFor(() => expect(result.current.loading).toBe(false));
    expect(result.current.user).toEqual(mockUser);

    await act(async () => {
      await result.current.logout();
    });

    expect(result.current.user).toBe(null);
    expect(result.current.loading).toBe(false);
  });

  it('should fetch user on initial load if authenticated', async () => {
    const mockUser = { id: '789', email: 'logged@example.com' };
    fetchMock.mockResponseOnce(JSON.stringify(mockUser), { status: 200 });

    const { result } = renderHook(() => useAuth(), { wrapper: AuthProvider });
    await waitFor(() => expect(result.current.loading).toBe(false));

    expect(result.current.user).toEqual(mockUser);
  });
});