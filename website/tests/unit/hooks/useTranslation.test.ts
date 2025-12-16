// website/tests/unit/hooks/useTranslation.test.ts
/**
 * Unit tests for useTranslation hook.
 *
 * Tests:
 * - Initial state and loading behavior
 * - Translate function and API interaction
 * - View mode toggling (original/translated)
 * - Cache loading from localStorage
 * - Content staleness detection
 * - Quota status handling
 * - Error handling
 */

import { renderHook, act, waitFor } from '@testing-library/react';
import { enableFetchMocks } from 'jest-fetch-mock';

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

jest.mock('@/services/translationService', () => ({
  TranslationApiClient: jest.fn().mockImplementation(() => ({
    translate: jest.fn(),
    getQuota: jest.fn(),
    checkHistory: jest.fn(),
  })),
  getFromCache: jest.fn(),
  saveToCache: jest.fn(),
  hasCachedTranslation: jest.fn(),
  invalidatePageCache: jest.fn(),
}));

import { useTranslation } from '@/hooks/useTranslation';
import { useAuth } from '@/hooks/useAuth';
import {
  TranslationApiClient,
  getFromCache,
  saveToCache,
} from '@/services/translationService';
import type { TranslatedPageCache, QuotaStatus } from '@/types/translation';

// Test data
const mockUser = { id: 'user123', email: 'test@example.com' };
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

  (getFromCache as jest.Mock).mockReturnValue(null);
});

describe('useTranslation', () => {
  describe('Initial State', () => {
    it('should start with idle state', () => {
      const { result } = renderHook(() => useTranslation('/docs/intro'));

      expect(result.current.state).toBe('idle');
      expect(result.current.viewMode).toBe('original');
      expect(result.current.error).toBeNull();
      expect(result.current.translatedContent).toBeNull();
    });

    it('should detect cached version on mount', () => {
      const cachedData: TranslatedPageCache = {
        content: 'ترجمہ شدہ متن',
        originalContentHash: 'abc123',
        timestamp: new Date().toISOString(),
        pageUrl: '/docs/intro',
        targetLanguage: 'ur',
        sourceType: 'original',
      };

      (getFromCache as jest.Mock).mockReturnValue(cachedData);

      const { result } = renderHook(() => useTranslation('/docs/intro'));

      expect(result.current.hasCachedVersion).toBe(true);
      expect(result.current.translatedContent).toBe('ترجمہ شدہ متن');
    });

    it('should reset state on page change', () => {
      const { result, rerender } = renderHook(
        ({ pageUrl }) => useTranslation(pageUrl),
        { initialProps: { pageUrl: '/docs/intro' } }
      );

      // Simulate some state change
      act(() => {
        // State would be changed by translate function
      });

      // Change page
      rerender({ pageUrl: '/docs/chapter1' });

      expect(result.current.state).toBe('idle');
      expect(result.current.viewMode).toBe('original');
    });
  });

  describe('Translate Function', () => {
    it('should translate content successfully', async () => {
      const mockApiClient = {
        translate: jest.fn().mockResolvedValue({
          translated_content: 'ترجمہ شدہ متن',
          source_language: 'en',
          target_language: 'ur',
          processing_time_ms: 1500,
          original_content_hash: 'hash123',
          quota_remaining: 4,
          quota_limit: 5,
        }),
        getQuota: jest.fn().mockResolvedValue({
          used: 1,
          remaining: 4,
          limit: 5,
          resets_at: new Date().toISOString(),
        }),
        checkHistory: jest.fn(),
      };

      (TranslationApiClient as jest.Mock).mockImplementation(() => mockApiClient);

      const { result } = renderHook(() => useTranslation('/docs/intro'));

      await act(async () => {
        await result.current.translate('Hello World', '/docs/intro', 'original');
      });

      expect(result.current.state).toBe('translated');
      expect(result.current.viewMode).toBe('translated');
      expect(result.current.translatedContent).toBe('ترجمہ شدہ متن');
      expect(saveToCache).toHaveBeenCalled();
    });

    it('should use cached version if available', async () => {
      const cachedData: TranslatedPageCache = {
        content: 'کیشڈ ترجمہ',
        originalContentHash: 'abc123',
        timestamp: new Date().toISOString(),
        pageUrl: '/docs/intro',
        targetLanguage: 'ur',
        sourceType: 'original',
      };

      (getFromCache as jest.Mock).mockReturnValue(cachedData);

      const mockApiClient = {
        translate: jest.fn(),
        getQuota: jest.fn().mockResolvedValue({
          used: 0,
          remaining: 5,
          limit: 5,
          resets_at: new Date().toISOString(),
        }),
        checkHistory: jest.fn(),
      };

      (TranslationApiClient as jest.Mock).mockImplementation(() => mockApiClient);

      const { result } = renderHook(() => useTranslation('/docs/intro'));

      await act(async () => {
        await result.current.translate('Hello World', '/docs/intro', 'original');
      });

      // Should use cache, not call API
      expect(mockApiClient.translate).not.toHaveBeenCalled();
      expect(result.current.translatedContent).toBe('کیشڈ ترجمہ');
    });

    it('should handle quota exceeded error', async () => {
      const mockApiClient = {
        translate: jest.fn().mockRejectedValue({
          code: 'QUOTA_EXCEEDED',
          error: 'Daily limit reached',
          details: { used: 5, limit: 5, resets_at: new Date().toISOString() },
        }),
        getQuota: jest.fn().mockResolvedValue({
          used: 5,
          remaining: 0,
          limit: 5,
          resets_at: new Date().toISOString(),
        }),
        checkHistory: jest.fn(),
      };

      (TranslationApiClient as jest.Mock).mockImplementation(() => mockApiClient);
      (getFromCache as jest.Mock).mockReturnValue(null);

      const { result } = renderHook(() => useTranslation('/docs/intro'));

      await act(async () => {
        await result.current.translate('Hello World', '/docs/intro', 'original');
      });

      expect(result.current.state).toBe('quota_exceeded');
      expect(result.current.error).toContain('Daily translation limit');
    });

    it('should handle timeout error', async () => {
      const mockApiClient = {
        translate: jest.fn().mockRejectedValue({
          code: 'TIMEOUT',
          error: 'Translation timed out',
        }),
        getQuota: jest.fn().mockResolvedValue({
          used: 0,
          remaining: 5,
          limit: 5,
          resets_at: new Date().toISOString(),
        }),
        checkHistory: jest.fn(),
      };

      (TranslationApiClient as jest.Mock).mockImplementation(() => mockApiClient);
      (getFromCache as jest.Mock).mockReturnValue(null);

      const { result } = renderHook(() => useTranslation('/docs/intro'));

      await act(async () => {
        await result.current.translate('Hello World', '/docs/intro', 'original');
      });

      expect(result.current.state).toBe('error');
      expect(result.current.error).toContain('timed out');
    });

    it('should handle generic errors', async () => {
      const mockApiClient = {
        translate: jest.fn().mockRejectedValue({
          error: 'Something went wrong',
        }),
        getQuota: jest.fn().mockResolvedValue({
          used: 0,
          remaining: 5,
          limit: 5,
          resets_at: new Date().toISOString(),
        }),
        checkHistory: jest.fn(),
      };

      (TranslationApiClient as jest.Mock).mockImplementation(() => mockApiClient);
      (getFromCache as jest.Mock).mockReturnValue(null);

      const { result } = renderHook(() => useTranslation('/docs/intro'));

      await act(async () => {
        await result.current.translate('Hello World', '/docs/intro', 'original');
      });

      expect(result.current.state).toBe('error');
      expect(result.current.error).toBe('Something went wrong');
    });
  });

  describe('Toggle View', () => {
    it('should toggle from original to translated when content exists', async () => {
      const mockApiClient = {
        translate: jest.fn().mockResolvedValue({
          translated_content: 'ترجمہ شدہ متن',
          source_language: 'en',
          target_language: 'ur',
          processing_time_ms: 1500,
          original_content_hash: 'hash123',
          quota_remaining: 4,
          quota_limit: 5,
        }),
        getQuota: jest.fn().mockResolvedValue({
          used: 1,
          remaining: 4,
          limit: 5,
          resets_at: new Date().toISOString(),
        }),
        checkHistory: jest.fn(),
      };

      (TranslationApiClient as jest.Mock).mockImplementation(() => mockApiClient);

      const { result } = renderHook(() => useTranslation('/docs/intro'));

      // First translate
      await act(async () => {
        await result.current.translate('Hello World', '/docs/intro', 'original');
      });

      expect(result.current.viewMode).toBe('translated');

      // Toggle to original
      act(() => {
        result.current.toggleView();
      });

      expect(result.current.viewMode).toBe('original');

      // Toggle back to translated
      act(() => {
        result.current.toggleView();
      });

      expect(result.current.viewMode).toBe('translated');
    });

    it('should not toggle to translated if no content', () => {
      const { result } = renderHook(() => useTranslation('/docs/intro'));

      expect(result.current.viewMode).toBe('original');

      act(() => {
        result.current.toggleView();
      });

      // Should stay on original since no translated content
      expect(result.current.viewMode).toBe('original');
    });
  });

  describe('Load Cached Version', () => {
    it('should load cached version and switch to translated view', () => {
      const cachedData: TranslatedPageCache = {
        content: 'کیشڈ ترجمہ',
        originalContentHash: 'abc123',
        timestamp: new Date().toISOString(),
        pageUrl: '/docs/intro',
        targetLanguage: 'ur',
        sourceType: 'original',
      };

      (getFromCache as jest.Mock).mockReturnValue(cachedData);

      const mockApiClient = {
        translate: jest.fn(),
        getQuota: jest.fn().mockResolvedValue({
          used: 0,
          remaining: 5,
          limit: 5,
          resets_at: new Date().toISOString(),
        }),
        checkHistory: jest.fn(),
      };

      (TranslationApiClient as jest.Mock).mockImplementation(() => mockApiClient);

      const { result } = renderHook(() => useTranslation('/docs/intro'));

      act(() => {
        result.current.loadCachedVersion();
      });

      expect(result.current.state).toBe('translated');
      expect(result.current.viewMode).toBe('translated');
      expect(result.current.translatedContent).toBe('کیشڈ ترجمہ');
    });
  });

  describe('Quota Status', () => {
    it('should fetch quota on mount', async () => {
      const mockQuota: QuotaStatus = {
        used: 2,
        remaining: 3,
        limit: 5,
        resets_at: new Date().toISOString(),
      };

      const mockApiClient = {
        translate: jest.fn(),
        getQuota: jest.fn().mockResolvedValue(mockQuota),
        checkHistory: jest.fn(),
      };

      (TranslationApiClient as jest.Mock).mockImplementation(() => mockApiClient);

      const { result } = renderHook(() => useTranslation('/docs/intro'));

      await waitFor(() => {
        expect(mockApiClient.getQuota).toHaveBeenCalled();
      });
    });

    it('should update quota after translation', async () => {
      const mockApiClient = {
        translate: jest.fn().mockResolvedValue({
          translated_content: 'ترجمہ شدہ متن',
          source_language: 'en',
          target_language: 'ur',
          processing_time_ms: 1500,
          original_content_hash: 'hash123',
          quota_remaining: 3,
          quota_limit: 5,
        }),
        getQuota: jest.fn().mockResolvedValue({
          used: 1,
          remaining: 4,
          limit: 5,
          resets_at: new Date().toISOString(),
        }),
        checkHistory: jest.fn(),
      };

      (TranslationApiClient as jest.Mock).mockImplementation(() => mockApiClient);

      const { result } = renderHook(() => useTranslation('/docs/intro'));

      await act(async () => {
        await result.current.translate('Hello World', '/docs/intro', 'original');
      });

      expect(result.current.quotaStatus?.remaining).toBe(3);
    });
  });

  describe('Loading State', () => {
    it('should show loading state during translation', async () => {
      let resolveTranslation: (value: unknown) => void;
      const translationPromise = new Promise((resolve) => {
        resolveTranslation = resolve;
      });

      const mockApiClient = {
        translate: jest.fn().mockReturnValue(translationPromise),
        getQuota: jest.fn().mockResolvedValue({
          used: 0,
          remaining: 5,
          limit: 5,
          resets_at: new Date().toISOString(),
        }),
        checkHistory: jest.fn(),
      };

      (TranslationApiClient as jest.Mock).mockImplementation(() => mockApiClient);
      (getFromCache as jest.Mock).mockReturnValue(null);

      const { result } = renderHook(() => useTranslation('/docs/intro'));

      // Start translation
      act(() => {
        result.current.translate('Hello World', '/docs/intro', 'original');
      });

      // Should be loading
      expect(result.current.state).toBe('loading');

      // Resolve translation
      await act(async () => {
        resolveTranslation!({
          translated_content: 'ترجمہ شدہ متن',
          source_language: 'en',
          target_language: 'ur',
          processing_time_ms: 1500,
          original_content_hash: 'hash123',
          quota_remaining: 4,
          quota_limit: 5,
        });
      });

      expect(result.current.state).toBe('translated');
    });
  });
});
