// website/src/services/translationService.ts
/**
 * Translation API service and localStorage cache manager.
 */

import type {
  TranslationRequest,
  TranslationResponse,
  QuotaStatus,
  TranslationHistoryCheck,
  TranslatedPageCache,
  TranslationError,
  TranslationSourceType,
} from '../types/translation';

// Constants
const CACHE_PREFIX = 'translated_ur_';
const CACHE_EXPIRY_DAYS = 30;
const MAX_CACHE_ENTRIES = 50;
const API_TIMEOUT_MS = 30000;

// ============================================================================
// localStorage Cache Functions
// ============================================================================

/**
 * Get cache key for a page.
 * Includes sourceType to differentiate between original and personalized translations.
 */
function getCacheKey(pageUrl: string, sourceType: TranslationSourceType = 'original'): string {
  return `${CACHE_PREFIX}${sourceType}_${encodeURIComponent(pageUrl)}`;
}

/**
 * Get all translation cache keys.
 */
function getAllCacheKeys(): string[] {
  const keys: string[] = [];
  for (let i = 0; i < localStorage.length; i++) {
    const key = localStorage.key(i);
    if (key?.startsWith(CACHE_PREFIX)) {
      keys.push(key);
    }
  }
  return keys;
}

/**
 * Check if a cache entry is expired.
 */
function isExpired(timestamp: string): boolean {
  const cacheDate = new Date(timestamp);
  const now = new Date();
  const diffDays = (now.getTime() - cacheDate.getTime()) / (1000 * 60 * 60 * 24);
  return diffDays > CACHE_EXPIRY_DAYS;
}

/**
 * Evict oldest entries when cache limit is reached (LRU eviction).
 */
function evictOldestIfNeeded(): void {
  const entries: { key: string; timestamp: string }[] = [];

  for (let i = 0; i < localStorage.length; i++) {
    const key = localStorage.key(i);
    if (key?.startsWith(CACHE_PREFIX)) {
      try {
        const data = JSON.parse(localStorage.getItem(key) || '{}') as TranslatedPageCache;
        entries.push({ key, timestamp: data.timestamp || '1970-01-01' });
      } catch {
        entries.push({ key, timestamp: '1970-01-01' });
      }
    }
  }

  if (entries.length >= MAX_CACHE_ENTRIES) {
    // Sort by timestamp, remove oldest
    entries.sort((a, b) => new Date(a.timestamp).getTime() - new Date(b.timestamp).getTime());
    localStorage.removeItem(entries[0].key);
  }
}

/**
 * Save translated content to localStorage cache.
 */
export function saveToCache(
  pageUrl: string,
  content: string,
  originalContentHash: string,
  sourceType: TranslationSourceType = 'original',
): void {
  try {
    evictOldestIfNeeded();
    const key = getCacheKey(pageUrl, sourceType);
    const now = new Date().toISOString();
    const entry: TranslatedPageCache = {
      content,
      originalContentHash,
      timestamp: now,
      pageUrl,
      targetLanguage: 'ur',
      sourceType,
    };
    localStorage.setItem(key, JSON.stringify(entry));
  } catch (error) {
    console.error('[translationService] Failed to save to cache:', error);
  }
}

/**
 * Get translated content from localStorage cache.
 * Returns null if not found or expired.
 */
export function getFromCache(
  pageUrl: string,
  sourceType: TranslationSourceType = 'original',
): TranslatedPageCache | null {
  try {
    const key = getCacheKey(pageUrl, sourceType);
    const data = localStorage.getItem(key);
    if (!data) return null;

    const entry = JSON.parse(data) as TranslatedPageCache;

    // Check expiry
    if (isExpired(entry.timestamp)) {
      localStorage.removeItem(key);
      return null;
    }

    // Ensure sourceType is set for backward compatibility
    if (!entry.sourceType) {
      entry.sourceType = 'original';
    }

    return entry;
  } catch {
    return null;
  }
}

/**
 * Check if a page has cached translation.
 */
export function hasCachedTranslation(
  pageUrl: string,
  sourceType: TranslationSourceType = 'original',
): boolean {
  const cached = getFromCache(pageUrl, sourceType);
  return cached !== null;
}

/**
 * Invalidate a specific page's cache.
 */
export function invalidatePageCache(
  pageUrl: string,
  sourceType: TranslationSourceType = 'original',
): void {
  const key = getCacheKey(pageUrl, sourceType);
  localStorage.removeItem(key);
}

/**
 * Invalidate both original and personalized caches for a page.
 */
export function invalidateAllPageCaches(pageUrl: string): void {
  invalidatePageCache(pageUrl, 'original');
  invalidatePageCache(pageUrl, 'personalized');
}

/**
 * Clear all translation cache entries.
 */
export function clearAllCache(): void {
  getAllCacheKeys().forEach((key) => localStorage.removeItem(key));
}

// ============================================================================
// API Client
// ============================================================================

/**
 * Translation API client class.
 */
export class TranslationApiClient {
  private baseUrl: string;
  private getToken: () => string | null;

  constructor(baseUrl: string, getToken: () => string | null) {
    this.baseUrl = baseUrl;
    this.getToken = getToken;
  }

  private async fetch<T>(
    endpoint: string,
    options: RequestInit = {},
  ): Promise<T> {
    const token = this.getToken();
    const headers: Record<string, string> = {
      'Content-Type': 'application/json',
    };

    // Add auth header if token available (optional for translation)
    if (token) {
      headers['Authorization'] = `Bearer ${token}`;
    }

    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), API_TIMEOUT_MS);

    try {
      const response = await fetch(`${this.baseUrl}${endpoint}`, {
        ...options,
        headers: {
          ...headers,
          ...options.headers,
        },
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        const error = (await response.json()) as TranslationError;
        throw error;
      }

      return response.json();
    } catch (error) {
      clearTimeout(timeoutId);
      if (error instanceof Error && error.name === 'AbortError') {
        throw { error: 'Translation timed out', code: 'TIMEOUT' };
      }
      throw error;
    }
  }

  /**
   * Translate page content to Urdu.
   */
  async translate(request: TranslationRequest): Promise<TranslationResponse> {
    const response = await this.fetch<{
      translated_content: string;
      source_language: string;
      target_language: string;
      processing_time_ms: number;
      original_content_hash: string;
      quota_remaining: number;
      quota_limit: number;
    }>('/api/translate/urdu', {
      method: 'POST',
      body: JSON.stringify(request),
    });

    return {
      translated_content: response.translated_content,
      source_language: response.source_language,
      target_language: response.target_language,
      processing_time_ms: response.processing_time_ms,
      original_content_hash: response.original_content_hash,
      quota_remaining: response.quota_remaining,
      quota_limit: response.quota_limit,
    };
  }

  /**
   * Get current quota status.
   */
  async getQuota(): Promise<QuotaStatus> {
    const response = await this.fetch<{
      used: number;
      remaining: number;
      limit: number;
      resets_at: string;
    }>('/api/translate/quota');

    return {
      used: response.used,
      remaining: response.remaining,
      limit: response.limit,
      resets_at: response.resets_at,
    };
  }

  /**
   * Check translation history for a page.
   */
  async checkHistory(
    pageUrl: string,
    currentContentHash?: string,
  ): Promise<TranslationHistoryCheck> {
    const params = new URLSearchParams({ page_url: pageUrl });
    if (currentContentHash) {
      params.append('current_content_hash', currentContentHash);
    }

    const response = await this.fetch<{
      has_translation: boolean;
      original_content_hash?: string;
      translated_at?: string;
      content_changed: boolean;
    }>(`/api/translate/history?${params.toString()}`);

    return {
      has_translation: response.has_translation,
      original_content_hash: response.original_content_hash,
      translated_at: response.translated_at,
      content_changed: response.content_changed,
    };
  }
}
