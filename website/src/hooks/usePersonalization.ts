// website/src/hooks/usePersonalization.ts
/**
 * React hook for page personalization.
 *
 * Manages personalization state, API calls, and localStorage caching.
 */

import { useState, useCallback, useEffect, useRef, useMemo } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useAuth } from './useAuth';
import { useProfile } from './useProfile.tsx';
import {
  PersonalizationApiClient,
  getFromCache,
  saveToCache,
  hasCachedPersonalization,
  invalidateAllCacheForUser,
  invalidatePageCache,
} from '../services/personalizationService';
import { computeProfileHash } from '../utils/hashUtils';
import type {
  PersonalizationState,
  ViewMode,
  QuotaStatus,
  PersonalizedPageCache,
} from '../types/personalization';

export interface UsePersonalizationResult {
  // State
  state: PersonalizationState;
  viewMode: ViewMode;
  error: string | null;
  quotaStatus: QuotaStatus | null;

  // Content
  personalizedContent: string | null;
  originalContent: string | null;

  // Cache info
  hasCachedVersion: boolean;
  isProfileStale: boolean;

  // Actions
  personalize: (pageContent: string, pageUrl: string) => Promise<void>;
  toggleView: () => void;
  loadCachedVersion: () => void;
  refreshQuota: () => Promise<void>;
}

export function usePersonalization(pageUrl: string): UsePersonalizationResult {
  const { siteConfig } = useDocusaurusContext();
  const { token, user } = useAuth();
  const { profile } = useProfile();

  const API_BASE_URL =
    (siteConfig.customFields?.backendUrl as string) || 'http://localhost:8000';

  // State
  const [state, setState] = useState<PersonalizationState>('idle');
  const [viewMode, setViewMode] = useState<ViewMode>('original');
  const [error, setError] = useState<string | null>(null);
  const [quotaStatus, setQuotaStatus] = useState<QuotaStatus | null>(null);
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
  const [originalContent, setOriginalContent] = useState<string | null>(null);
  const [hasCachedVersion, setHasCachedVersion] = useState(false);
  const [isProfileStale, setIsProfileStale] = useState(false);

  // Refs
  const abortControllerRef = useRef<AbortController | null>(null);
  const previousProfileHashRef = useRef<string | null>(null);
  const isServerHistoryAvailableRef = useRef<boolean>(true);

  // API client
  const apiClient = useMemo(() => {
    return new PersonalizationApiClient(API_BASE_URL, () => token);
  }, [API_BASE_URL, token]);

  // Helper to get/set the last personalized page from sessionStorage
  // Using sessionStorage instead of useRef because the component remounts when DOM changes
  const getLastPersonalizedPage = () => sessionStorage.getItem('lastPersonalizedPage');
  const setLastPersonalizedPage = (url: string | null) => {
    if (url) {
      sessionStorage.setItem('lastPersonalizedPage', url);
    } else {
      sessionStorage.removeItem('lastPersonalizedPage');
    }
  };

  // Reset state and check cache on page change
  useEffect(() => {
    // Cancel any in-flight request when page changes
    if (abortControllerRef.current) {
      abortControllerRef.current.abort();
      abortControllerRef.current = null;
    }

    const lastPersonalizedPage = getLastPersonalizedPage();

    // Only reset to idle if we're navigating to a DIFFERENT page
    // If we just personalized this page, don't reset the state
    if (lastPersonalizedPage !== pageUrl) {
      setState('idle');
      setViewMode('original');
      setError(null);
      // Clear the storage since we're on a different page now
      setLastPersonalizedPage(null);
    } else if (lastPersonalizedPage === pageUrl) {
      // We just personalized this page - restore personalized state
      console.log('[usePersonalization] Restoring personalized state after remount');
      setState('personalized');
      setViewMode('personalized');
    }

    if (!user?.id || !profile) {
      setHasCachedVersion(false);
      setPersonalizedContent(null);
      setOriginalContent(null);
      return;
    }

    const cached = getFromCache(pageUrl, user.id);
    if (cached && cached.content) {
      // We have a valid cached personalization
      setHasCachedVersion(true);
      setPersonalizedContent(cached.content);
      setOriginalContent(cached.originalContent);
      // Don't auto-apply - let user choose via "View Personalized" button
      // Keep state as 'idle' so they see the "cached version available" UI

      // Check if profile has changed since personalization
      computeProfileHash(profile).then((currentHash) => {
        setIsProfileStale(currentHash !== cached.profileHash);
      });
    } else {
      setHasCachedVersion(false);
      setPersonalizedContent(null);
      setOriginalContent(null);
      setIsProfileStale(false);
    }
  }, [pageUrl, user?.id, profile]);

  // Fetch quota on mount
  useEffect(() => {
    if (token && user) {
      refreshQuota();
    }
  }, [token, user]);

  // Profile change detection (T061-T062): Invalidate cache on profile update
  useEffect(() => {
    if (!user?.id || !profile) return;

    const detectProfileChange = async () => {
      const currentHash = await computeProfileHash(profile);

      // Check if profile has changed
      if (previousProfileHashRef.current !== null && previousProfileHashRef.current !== currentHash) {
        // Profile changed - invalidate all cached personalizations for this user
        console.log('[usePersonalization] Profile changed, invalidating cache');
        invalidateAllCacheForUser(user.id);

        // Reset state to reflect cache invalidation
        setHasCachedVersion(false);
        setPersonalizedContent(null);
        setIsProfileStale(false);
      }

      // Update the reference
      previousProfileHashRef.current = currentHash;
    };

    detectProfileChange();
  }, [profile, user?.id]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (abortControllerRef.current) {
        abortControllerRef.current.abort();
      }
    };
  }, []);

  const refreshQuota = useCallback(async () => {
    if (!token) return;
    try {
      const quota = await apiClient.getQuota();
      setQuotaStatus(quota);
    } catch (err) {
      console.error('Failed to fetch quota:', err);
    }
  }, [apiClient, token]);

  const personalize = useCallback(
    async (pageContent: string, currentPageUrl: string) => {
      console.log('[usePersonalization] personalize called:', {
        hasToken: !!token,
        hasUser: !!user?.id,
        hasProfile: !!profile,
        contentLength: pageContent?.length,
        pageUrl: currentPageUrl,
      });

      if (!token || !user?.id || !profile) {
        console.error('[usePersonalization] Missing auth/profile:', { token: !!token, userId: user?.id, profile: !!profile });
        setError('Please log in and complete your profile to personalize content');
        setState('error');
        return;
      }

      // Check if we already have a cached version with same profile
      const cached = getFromCache(currentPageUrl, user.id);
      const currentProfileHash = await computeProfileHash(profile);

      if (cached && cached.profileHash === currentProfileHash) {
        // T060: Check if original content has changed since personalization
        const { computeContentHash } = await import('../utils/hashUtils');
        const currentContentHash = await computeContentHash(pageContent);

        if (cached.originalContentHash !== currentContentHash) {
          // Original content has changed - invalidate cache for this page
          console.log('[usePersonalization] Original content changed, invalidating page cache');
          invalidatePageCache(currentPageUrl, user.id);
          setHasCachedVersion(false);
          // Continue to re-personalize instead of using stale cache
        } else {
          // Use cached version
          setPersonalizedContent(cached.content);
          setOriginalContent(cached.originalContent);
          setViewMode('personalized');
          setState('personalized');
          return;
        }
      }

      // Cancel any existing request
      if (abortControllerRef.current) {
        abortControllerRef.current.abort();
      }
      abortControllerRef.current = new AbortController();

      setState('loading');
      setError(null);
      setOriginalContent(pageContent);

      // Determine if this is a free re-personalization
      const isFreeRepersonalization = cached !== null;

      console.log('[usePersonalization] Making API call to personalize...');

      try {
        const response = await apiClient.personalize({
          pageUrl: currentPageUrl,
          pageContent,
          isFreeRepersonalization,
        });
        console.log('[usePersonalization] API response received:', {
          hasContent: !!response.personalizedContent,
          contentLength: response.personalizedContent?.length,
          profileHash: response.profileHash,
        });

        // Save to cache
        saveToCache(
          currentPageUrl,
          user.id,
          response.personalizedContent,
          pageContent,
          response.profileHash,
          response.originalContentHash
        );

        setPersonalizedContent(response.personalizedContent);
        setQuotaStatus((prev) =>
          prev
            ? { ...prev, remaining: response.quotaRemaining, used: prev.limit - response.quotaRemaining }
            : null
        );
        setViewMode('personalized');
        setState('personalized');
        setHasCachedVersion(true);
        setIsProfileStale(false);
        // Mark this page as just personalized to prevent state reset on re-render
        setLastPersonalizedPage(currentPageUrl);
      } catch (err: unknown) {
        console.error('[usePersonalization] API call failed:', err);

        if (err instanceof Error && err.name === 'AbortError') {
          // Request was cancelled
          return;
        }

        const apiError = err as { error?: string; message?: string; quotaStatus?: QuotaStatus };
        console.error('[usePersonalization] Error details:', apiError);

        if (apiError.error === 'quota_exceeded') {
          setState('quota_exceeded');
          if (apiError.quotaStatus) {
            setQuotaStatus(apiError.quotaStatus);
          }
          setError('Daily personalization limit reached. Try again tomorrow!');
        } else if (apiError.error === 'profile_incomplete') {
          setState('error');
          setError('Please complete your profile to personalize content');
        } else {
          setState('error');
          setError(apiError.message || 'Failed to personalize content. Please try again.');
        }
      }
    },
    [token, user?.id, profile, apiClient]
  );

  const toggleView = useCallback(() => {
    console.log('[usePersonalization] toggleView called:', {
      viewMode,
      hasPersonalizedContent: !!personalizedContent,
      hasOriginalContent: !!originalContent,
    });
    if (viewMode === 'original' && personalizedContent) {
      setViewMode('personalized');
    } else if (viewMode === 'personalized') {
      // Always allow toggling back to original when in personalized mode
      // The original content is preserved in the DOM by PersonalizationBar
      setViewMode('original');
    } else {
      console.warn('[usePersonalization] toggleView: cannot toggle, missing content');
    }
  }, [viewMode, personalizedContent, originalContent]);

  const loadCachedVersion = useCallback(() => {
    console.log('[usePersonalization] loadCachedVersion called', { userId: user?.id, pageUrl });
    if (!user?.id) {
      console.log('[usePersonalization] No user ID, returning');
      return;
    }

    const cached = getFromCache(pageUrl, user.id);
    console.log('[usePersonalization] Cached data:', {
      found: !!cached,
      contentLength: cached?.content?.length,
      originalContentLength: cached?.originalContent?.length,
    });

    if (cached) {
      setPersonalizedContent(cached.content);
      setOriginalContent(cached.originalContent);
      setViewMode('personalized');
      setState('personalized');
      console.log('[usePersonalization] State updated to personalized');
    }
  }, [pageUrl, user?.id]);

  return {
    state,
    viewMode,
    error,
    quotaStatus,
    personalizedContent,
    originalContent,
    hasCachedVersion,
    isProfileStale,
    personalize,
    toggleView,
    loadCachedVersion,
    refreshQuota,
  };
}
