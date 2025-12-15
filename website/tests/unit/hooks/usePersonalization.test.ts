// website/tests/unit/hooks/usePersonalization.test.ts
/**
 * Unit tests for usePersonalization hook.
 *
 * Tests:
 * - Initial state and loading behavior
 * - Personalize function and API interaction
 * - View mode toggling (original/personalized)
 * - Cache loading from localStorage
 * - Profile staleness detection
 * - Quota status handling
 * - Error handling
 */

import { renderHook, act, waitFor } from '@testing-library/react';
import { enableFetchMocks } from 'jest-fetch-mock';
import React from 'react';

enableFetchMocks();

// Mock the dependencies
jest.mock('@docusaurus/useDocusaurusContext', () => ({
  __esModule: true,
  default: jest.fn(() => ({
    siteConfig: {
      customFields: {
        backendUrl: 'http://localhost:8000',
      },
    },
  })),
}));

jest.mock('@/hooks/useAuth', () => ({
  useAuth: jest.fn(),
}));

jest.mock('@/hooks/useProfile.tsx', () => ({
  useProfile: jest.fn(),
}));

jest.mock('@/services/personalizationService', () => ({
  PersonalizationApiClient: jest.fn().mockImplementation(() => ({
    personalize: jest.fn(),
    getQuota: jest.fn(),
    getHistory: jest.fn(),
    getPageHistory: jest.fn(),
  })),
  getFromCache: jest.fn(),
  saveToCache: jest.fn(),
  hasCachedPersonalization: jest.fn(),
}));

jest.mock('@/utils/hashUtils', () => ({
  computeProfileHash: jest.fn(),
}));

import { usePersonalization } from '@/hooks/usePersonalization';
import { useAuth } from '@/hooks/useAuth';
import { useProfile } from '@/hooks/useProfile.tsx';
import {
  PersonalizationApiClient,
  getFromCache,
  saveToCache,
} from '@/services/personalizationService';
import { computeProfileHash } from '@/utils/hashUtils';
import type { PersonalizedPageCache, QuotaStatus } from '@/types/personalization';

// Test data
const mockUser = { id: 'user123', email: 'test@example.com' };
const mockProfile = {
  user_id: 'user123',
  tech_background: 'intermediate',
  learning_mode: 'visual',
  learning_speed: 'balanced',
};
const mockToken = 'mock-jwt-token';

// Reset mocks before each test
beforeEach(() => {
  jest.clearAllMocks();
  fetchMock.resetMocks();

  // Default mock implementations
  (useAuth as jest.Mock).mockReturnValue({
    user: mockUser,
    token: mockToken,
    loading: false,
  });

  (useProfile as jest.Mock).mockReturnValue({
    profile: mockProfile,
    loadingProfile: false,
  });

  (getFromCache as jest.Mock).mockReturnValue(null);
  (computeProfileHash as jest.Mock).mockResolvedValue('profile_hash_123');
});

describe('usePersonalization', () => {
  describe('Initial State', () => {
    it('should initialize with idle state', () => {
      const { result } = renderHook(() => usePersonalization('/docs/intro'));

      expect(result.current.state).toBe('idle');
      expect(result.current.viewMode).toBe('original');
      expect(result.current.error).toBeNull();
      expect(result.current.personalizedContent).toBeNull();
    });

    it('should set hasCachedVersion to false when no cache exists', () => {
      (getFromCache as jest.Mock).mockReturnValue(null);

      const { result } = renderHook(() => usePersonalization('/docs/intro'));

      expect(result.current.hasCachedVersion).toBe(false);
    });

    it('should set hasCachedVersion to true when cache exists', async () => {
      const mockCache: PersonalizedPageCache = {
        content: 'Personalized content',
        originalContent: 'Original content',
        profileHash: 'profile_hash_123',
        originalContentHash: 'content_hash_123',
        timestamp: new Date().toISOString(),
        pageUrl: '/docs/intro',
        lastAccessed: new Date().toISOString(),
      };
      (getFromCache as jest.Mock).mockReturnValue(mockCache);

      const { result } = renderHook(() => usePersonalization('/docs/intro'));

      await waitFor(() => {
        expect(result.current.hasCachedVersion).toBe(true);
      });
    });
  });

  describe('Profile Staleness Detection', () => {
    it('should detect stale profile when hashes differ', async () => {
      const mockCache: PersonalizedPageCache = {
        content: 'Personalized content',
        originalContent: 'Original content',
        profileHash: 'old_profile_hash',
        originalContentHash: 'content_hash_123',
        timestamp: new Date().toISOString(),
        pageUrl: '/docs/intro',
        lastAccessed: new Date().toISOString(),
      };
      (getFromCache as jest.Mock).mockReturnValue(mockCache);
      (computeProfileHash as jest.Mock).mockResolvedValue('new_profile_hash');

      const { result } = renderHook(() => usePersonalization('/docs/intro'));

      await waitFor(() => {
        expect(result.current.isProfileStale).toBe(true);
      });
    });

    it('should not be stale when hashes match', async () => {
      const mockCache: PersonalizedPageCache = {
        content: 'Personalized content',
        originalContent: 'Original content',
        profileHash: 'profile_hash_123',
        originalContentHash: 'content_hash_123',
        timestamp: new Date().toISOString(),
        pageUrl: '/docs/intro',
        lastAccessed: new Date().toISOString(),
      };
      (getFromCache as jest.Mock).mockReturnValue(mockCache);
      (computeProfileHash as jest.Mock).mockResolvedValue('profile_hash_123');

      const { result } = renderHook(() => usePersonalization('/docs/intro'));

      await waitFor(() => {
        expect(result.current.isProfileStale).toBe(false);
      });
    });
  });

  describe('Personalize Function', () => {
    it('should set loading state when personalizing', async () => {
      const mockApiClient = {
        personalize: jest.fn().mockImplementation(
          () =>
            new Promise((resolve) =>
              setTimeout(
                () =>
                  resolve({
                    personalizedContent: 'Personalized content',
                    profileHash: 'hash123',
                    originalContentHash: 'content_hash',
                    processingTimeMs: 1500,
                    quotaRemaining: 4,
                  }),
                100
              )
            )
        ),
        getQuota: jest.fn().mockResolvedValue({ limit: 5, used: 0, remaining: 5 }),
      };
      (PersonalizationApiClient as jest.Mock).mockImplementation(() => mockApiClient);

      const { result } = renderHook(() => usePersonalization('/docs/intro'));

      act(() => {
        result.current.personalize('Original page content', '/docs/intro');
      });

      expect(result.current.state).toBe('loading');
    });

    it('should set personalized state on success', async () => {
      const mockApiClient = {
        personalize: jest.fn().mockResolvedValue({
          personalizedContent: 'Personalized content',
          profileHash: 'hash123',
          originalContentHash: 'content_hash',
          processingTimeMs: 1500,
          quotaRemaining: 4,
        }),
        getQuota: jest.fn().mockResolvedValue({ limit: 5, used: 0, remaining: 5 }),
      };
      (PersonalizationApiClient as jest.Mock).mockImplementation(() => mockApiClient);

      const { result } = renderHook(() => usePersonalization('/docs/intro'));

      await act(async () => {
        await result.current.personalize('Original page content', '/docs/intro');
      });

      expect(result.current.state).toBe('personalized');
      expect(result.current.personalizedContent).toBe('Personalized content');
      expect(result.current.viewMode).toBe('personalized');
    });

    it('should save to cache after successful personalization', async () => {
      const mockApiClient = {
        personalize: jest.fn().mockResolvedValue({
          personalizedContent: 'Personalized content',
          profileHash: 'hash123',
          originalContentHash: 'content_hash',
          processingTimeMs: 1500,
          quotaRemaining: 4,
        }),
        getQuota: jest.fn().mockResolvedValue({ limit: 5, used: 0, remaining: 5 }),
      };
      (PersonalizationApiClient as jest.Mock).mockImplementation(() => mockApiClient);

      const { result } = renderHook(() => usePersonalization('/docs/intro'));

      await act(async () => {
        await result.current.personalize('Original page content', '/docs/intro');
      });

      expect(saveToCache).toHaveBeenCalledWith(
        '/docs/intro',
        'user123',
        'Personalized content',
        'Original page content',
        'hash123',
        'content_hash'
      );
    });

    it('should use cached version if profile hash matches', async () => {
      const mockCache: PersonalizedPageCache = {
        content: 'Cached personalized content',
        originalContent: 'Original content',
        profileHash: 'profile_hash_123',
        originalContentHash: 'content_hash',
        timestamp: new Date().toISOString(),
        pageUrl: '/docs/intro',
        lastAccessed: new Date().toISOString(),
      };
      (getFromCache as jest.Mock).mockReturnValue(mockCache);
      (computeProfileHash as jest.Mock).mockResolvedValue('profile_hash_123');

      const mockApiClient = {
        personalize: jest.fn(),
        getQuota: jest.fn().mockResolvedValue({ limit: 5, used: 0, remaining: 5 }),
      };
      (PersonalizationApiClient as jest.Mock).mockImplementation(() => mockApiClient);

      const { result } = renderHook(() => usePersonalization('/docs/intro'));

      await act(async () => {
        await result.current.personalize('Original content', '/docs/intro');
      });

      // Should use cache, not call API
      expect(mockApiClient.personalize).not.toHaveBeenCalled();
      expect(result.current.personalizedContent).toBe('Cached personalized content');
    });

    it('should set error state on API failure', async () => {
      const mockApiClient = {
        personalize: jest.fn().mockRejectedValue({
          error: 'server_error',
          message: 'Something went wrong',
        }),
        getQuota: jest.fn().mockResolvedValue({ limit: 5, used: 0, remaining: 5 }),
      };
      (PersonalizationApiClient as jest.Mock).mockImplementation(() => mockApiClient);

      const { result } = renderHook(() => usePersonalization('/docs/intro'));

      await act(async () => {
        await result.current.personalize('Original page content', '/docs/intro');
      });

      expect(result.current.state).toBe('error');
      expect(result.current.error).toBe('Something went wrong');
    });

    it('should set quota_exceeded state when quota exceeded', async () => {
      const mockApiClient = {
        personalize: jest.fn().mockRejectedValue({
          error: 'quota_exceeded',
          message: 'Daily limit reached',
          quotaStatus: { limit: 5, used: 5, remaining: 0 },
        }),
        getQuota: jest.fn().mockResolvedValue({ limit: 5, used: 5, remaining: 0 }),
      };
      (PersonalizationApiClient as jest.Mock).mockImplementation(() => mockApiClient);

      const { result } = renderHook(() => usePersonalization('/docs/intro'));

      await act(async () => {
        await result.current.personalize('Original page content', '/docs/intro');
      });

      expect(result.current.state).toBe('quota_exceeded');
    });

    it('should require authentication to personalize', async () => {
      (useAuth as jest.Mock).mockReturnValue({
        user: null,
        token: null,
        loading: false,
      });

      const { result } = renderHook(() => usePersonalization('/docs/intro'));

      await act(async () => {
        await result.current.personalize('Original content', '/docs/intro');
      });

      expect(result.current.state).toBe('error');
      expect(result.current.error).toContain('log in');
    });

    it('should require profile to personalize', async () => {
      (useProfile as jest.Mock).mockReturnValue({
        profile: null,
        loadingProfile: false,
      });

      const { result } = renderHook(() => usePersonalization('/docs/intro'));

      await act(async () => {
        await result.current.personalize('Original content', '/docs/intro');
      });

      expect(result.current.state).toBe('error');
      expect(result.current.error).toContain('profile');
    });
  });

  describe('Toggle View Function', () => {
    it('should toggle from original to personalized view', async () => {
      const mockCache: PersonalizedPageCache = {
        content: 'Personalized content',
        originalContent: 'Original content',
        profileHash: 'profile_hash_123',
        originalContentHash: 'content_hash',
        timestamp: new Date().toISOString(),
        pageUrl: '/docs/intro',
        lastAccessed: new Date().toISOString(),
      };
      (getFromCache as jest.Mock).mockReturnValue(mockCache);

      const mockApiClient = {
        personalize: jest.fn().mockResolvedValue({
          personalizedContent: 'Personalized content',
          profileHash: 'hash123',
          originalContentHash: 'content_hash',
          processingTimeMs: 1500,
          quotaRemaining: 4,
        }),
        getQuota: jest.fn().mockResolvedValue({ limit: 5, used: 0, remaining: 5 }),
      };
      (PersonalizationApiClient as jest.Mock).mockImplementation(() => mockApiClient);

      const { result } = renderHook(() => usePersonalization('/docs/intro'));

      // First personalize
      await act(async () => {
        await result.current.personalize('Original content', '/docs/intro');
      });

      expect(result.current.viewMode).toBe('personalized');

      // Toggle to original
      act(() => {
        result.current.toggleView();
      });

      expect(result.current.viewMode).toBe('original');

      // Toggle back to personalized
      act(() => {
        result.current.toggleView();
      });

      expect(result.current.viewMode).toBe('personalized');
    });
  });

  describe('Load Cached Version', () => {
    it('should load cached personalized content', async () => {
      const mockCache: PersonalizedPageCache = {
        content: 'Cached personalized content',
        originalContent: 'Original content',
        profileHash: 'profile_hash_123',
        originalContentHash: 'content_hash',
        timestamp: new Date().toISOString(),
        pageUrl: '/docs/intro',
        lastAccessed: new Date().toISOString(),
      };
      (getFromCache as jest.Mock).mockReturnValue(mockCache);

      const { result } = renderHook(() => usePersonalization('/docs/intro'));

      act(() => {
        result.current.loadCachedVersion();
      });

      expect(result.current.personalizedContent).toBe('Cached personalized content');
      expect(result.current.viewMode).toBe('personalized');
      expect(result.current.state).toBe('personalized');
    });
  });

  describe('Quota Refresh', () => {
    it('should fetch quota on mount when authenticated', async () => {
      const mockApiClient = {
        getQuota: jest.fn().mockResolvedValue({
          limit: 5,
          used: 2,
          remaining: 3,
          resetsAt: new Date().toISOString(),
        }),
      };
      (PersonalizationApiClient as jest.Mock).mockImplementation(() => mockApiClient);

      renderHook(() => usePersonalization('/docs/intro'));

      await waitFor(() => {
        expect(mockApiClient.getQuota).toHaveBeenCalled();
      });
    });

    it('should update quota status after refresh', async () => {
      const mockQuota: QuotaStatus = {
        limit: 5,
        used: 2,
        remaining: 3,
        resetsAt: new Date().toISOString(),
      };
      const mockApiClient = {
        getQuota: jest.fn().mockResolvedValue(mockQuota),
      };
      (PersonalizationApiClient as jest.Mock).mockImplementation(() => mockApiClient);

      const { result } = renderHook(() => usePersonalization('/docs/intro'));

      await waitFor(() => {
        expect(result.current.quotaStatus).toEqual(mockQuota);
      });
    });
  });

  describe('Page Navigation', () => {
    it('should reset state when page URL changes', async () => {
      const mockApiClient = {
        personalize: jest.fn().mockResolvedValue({
          personalizedContent: 'Personalized content',
          profileHash: 'hash123',
          originalContentHash: 'content_hash',
          processingTimeMs: 1500,
          quotaRemaining: 4,
        }),
        getQuota: jest.fn().mockResolvedValue({ limit: 5, used: 0, remaining: 5 }),
      };
      (PersonalizationApiClient as jest.Mock).mockImplementation(() => mockApiClient);

      const { result, rerender } = renderHook(
        ({ pageUrl }) => usePersonalization(pageUrl),
        { initialProps: { pageUrl: '/docs/intro' } }
      );

      // Personalize first page
      await act(async () => {
        await result.current.personalize('Original content', '/docs/intro');
      });

      expect(result.current.state).toBe('personalized');

      // Navigate to new page
      rerender({ pageUrl: '/docs/chapter1' });

      expect(result.current.state).toBe('idle');
      expect(result.current.viewMode).toBe('original');
    });
  });
});
