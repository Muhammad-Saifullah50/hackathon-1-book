// website/src/services/personalizationService.ts
/**
 * Personalization API service and localStorage cache manager.
 */

import type {
  PersonalizationRequest,
  PersonalizationResponse,
  QuotaStatus,
  PersonalizationHistoryResponse,
  PageHistoryResponse,
  PersonalizedPageCache,
  PersonalizationError,
} from '../types/personalization';

// Constants
const CACHE_PREFIX = 'personalized_';
const MAX_CACHED_PAGES = 20;

/**
 * Get cache key for a page.
 */
function getCacheKey(pageUrl: string, userId: string): string {
  return `${CACHE_PREFIX}${pageUrl}_${userId}`;
}

/**
 * Get all cache keys for a user.
 */
function getAllCacheKeys(userId: string): string[] {
  const keys: string[] = [];
  for (let i = 0; i < localStorage.length; i++) {
    const key = localStorage.key(i);
    if (key?.startsWith(CACHE_PREFIX) && key.endsWith(`_${userId}`)) {
      keys.push(key);
    }
  }
  return keys;
}

/**
 * Evict least recently used entries when cache limit is reached.
 */
function evictLRU(userId: string): void {
  const keys = getAllCacheKeys(userId);
  if (keys.length < MAX_CACHED_PAGES) return;

  // Sort by lastAccessed (oldest first)
  const entries = keys
    .map((key) => {
      try {
        const data = JSON.parse(localStorage.getItem(key) || '{}') as PersonalizedPageCache;
        return { key, lastAccessed: data.lastAccessed || data.timestamp };
      } catch {
        return { key, lastAccessed: '1970-01-01T00:00:00Z' };
      }
    })
    .sort(
      (a, b) => new Date(a.lastAccessed).getTime() - new Date(b.lastAccessed).getTime()
    );

  // Remove oldest entries until under limit
  const toRemove = entries.slice(0, entries.length - MAX_CACHED_PAGES + 1);
  toRemove.forEach((entry) => localStorage.removeItem(entry.key));
}

/**
 * Save personalized content to localStorage cache.
 */
export function saveToCache(
  pageUrl: string,
  userId: string,
  personalizedContent: string,
  originalContent: string,
  profileHash: string,
  originalContentHash: string
): void {
  try {
    evictLRU(userId);
    const key = getCacheKey(pageUrl, userId);
    const now = new Date().toISOString();
    const entry: PersonalizedPageCache = {
      content: personalizedContent,
      originalContent,
      profileHash,
      originalContentHash,
      timestamp: now,
      pageUrl,
      lastAccessed: now,
    };
    localStorage.setItem(key, JSON.stringify(entry));
  } catch (error) {
    console.error('Failed to save to cache:', error);
  }
}

/**
 * Get personalized content from localStorage cache.
 * Updates lastAccessed on read.
 */
export function getFromCache(pageUrl: string, userId: string): PersonalizedPageCache | null {
  try {
    const key = getCacheKey(pageUrl, userId);
    const data = localStorage.getItem(key);
    if (!data) return null;

    const entry = JSON.parse(data) as PersonalizedPageCache;
    // Update lastAccessed on read
    entry.lastAccessed = new Date().toISOString();
    localStorage.setItem(key, JSON.stringify(entry));

    return entry;
  } catch {
    return null;
  }
}

/**
 * Check if a page has cached personalization.
 */
export function hasCachedPersonalization(pageUrl: string, userId: string): boolean {
  const key = getCacheKey(pageUrl, userId);
  return localStorage.getItem(key) !== null;
}

/**
 * Invalidate all cached personalizations for a user.
 * Call this when profile is updated.
 */
export function invalidateAllCacheForUser(userId: string): void {
  getAllCacheKeys(userId).forEach((key) => localStorage.removeItem(key));
}

/**
 * Invalidate a specific page's cache.
 */
export function invalidatePageCache(pageUrl: string, userId: string): void {
  const key = getCacheKey(pageUrl, userId);
  localStorage.removeItem(key);
}

// ============================================================================
// API Client
// ============================================================================

/**
 * Personalization API client class.
 */
export class PersonalizationApiClient {
  private baseUrl: string;
  private getToken: () => string | null;

  constructor(baseUrl: string, getToken: () => string | null) {
    this.baseUrl = baseUrl;
    this.getToken = getToken;
  }

  private async fetch<T>(
    endpoint: string,
    options: RequestInit = {}
  ): Promise<T> {
    const token = this.getToken();
    if (!token) {
      throw new Error('No authentication token available');
    }

    const response = await fetch(`${this.baseUrl}${endpoint}`, {
      ...options,
      headers: {
        'Content-Type': 'application/json',
        Authorization: `Bearer ${token}`,
        ...options.headers,
      },
    });

    if (!response.ok) {
      const error = (await response.json()) as PersonalizationError;
      throw error;
    }

    return response.json();
  }

  /**
   * Personalize page content.
   */
  async personalize(request: PersonalizationRequest): Promise<PersonalizationResponse> {
    const response = await this.fetch<{
      personalized_content: string;
      profile_hash: string;
      original_content_hash: string;
      processing_time_ms: number;
      quota_remaining: number;
    }>('/personalization/personalize', {
      method: 'POST',
      body: JSON.stringify({
        page_url: request.pageUrl,
        page_content: request.pageContent,
        is_free_repersonalization: request.isFreeRepersonalization,
      }),
    });

    return {
      personalizedContent: response.personalized_content,
      profileHash: response.profile_hash,
      originalContentHash: response.original_content_hash,
      processingTimeMs: response.processing_time_ms,
      quotaRemaining: response.quota_remaining,
    };
  }

  /**
   * Get current quota status.
   */
  async getQuota(): Promise<QuotaStatus> {
    const response = await this.fetch<{
      limit: number;
      used: number;
      remaining: number;
      resets_at: string;
    }>('/personalization/quota');

    return {
      limit: response.limit,
      used: response.used,
      remaining: response.remaining,
      resetsAt: response.resets_at,
    };
  }

  /**
   * Get personalization history.
   */
  async getHistory(): Promise<PersonalizationHistoryResponse> {
    const response = await this.fetch<{
      pages: Array<{
        url: string;
        profile_hash: string;
        created_at: string;
      }>;
      current_profile_hash: string;
    }>('/personalization/history');

    return {
      pages: response.pages.map((p) => ({
        url: p.url,
        profileHash: p.profile_hash,
        createdAt: p.created_at,
      })),
      currentProfileHash: response.current_profile_hash,
    };
  }

  /**
   * Check if a specific page was personalized.
   */
  async getPageHistory(pageUrl: string): Promise<PageHistoryResponse> {
    const encodedUrl = encodeURIComponent(pageUrl);
    const response = await this.fetch<{
      found: boolean;
      profile_hash?: string;
      original_content_hash?: string;
      created_at?: string;
      is_stale?: boolean;
    }>(`/personalization/history/${encodedUrl}`);

    return {
      found: response.found,
      profileHash: response.profile_hash,
      originalContentHash: response.original_content_hash,
      createdAt: response.created_at,
      isStale: response.is_stale,
    };
  }
}
