// website/src/hooks/useTranslation.ts
/**
 * React hook for page translation to Urdu.
 *
 * Manages translation state, API calls, and localStorage caching.
 * Works for both authenticated and anonymous users.
 */

import { useState, useCallback, useEffect, useRef, useMemo } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useAuth } from './useAuth';
import {
  TranslationApiClient,
  getFromCache,
  saveToCache,
  hasCachedTranslation,
  invalidatePageCache,
} from '../services/translationService';
import type {
  TranslationState,
  ViewMode,
  QuotaStatus,
  UseTranslationReturn,
  TranslationSourceType,
} from '../types/translation';

export function useTranslation(pageUrl: string): UseTranslationReturn {
  const { siteConfig } = useDocusaurusContext();
  const { token } = useAuth();

  const API_BASE_URL =
    (siteConfig.customFields?.backendUrl as string) || 'http://localhost:8000';

  // State
  const [state, setState] = useState<TranslationState>('idle');
  const [viewMode, setViewMode] = useState<ViewMode>('original');
  const [error, setError] = useState<string | null>(null);
  const [quotaStatus, setQuotaStatus] = useState<QuotaStatus | null>(null);
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [hasCachedVersion, setHasCachedVersion] = useState(false);
  const [isContentStale, setIsContentStale] = useState(false);
  const [translationSourceType, setTranslationSourceType] = useState<TranslationSourceType | null>(null);

  // Refs
  const abortControllerRef = useRef<AbortController | null>(null);
  const currentSourceTypeRef = useRef<TranslationSourceType>('original');

  // API client - works without token for anonymous users
  const apiClient = useMemo(() => {
    return new TranslationApiClient(API_BASE_URL, () => token);
  }, [API_BASE_URL, token]);

  // Check cache on page change
  useEffect(() => {
    // Cancel any in-flight request when page changes
    if (abortControllerRef.current) {
      abortControllerRef.current.abort();
      abortControllerRef.current = null;
    }

    // Reset state
    setState('idle');
    setViewMode('original');
    setError(null);
    setTranslatedContent(null);
    setTranslationSourceType(null);
    currentSourceTypeRef.current = 'original';

    // Check for cached translation (check both original and personalized)
    // Prefer personalized cache if available
    const cachedPersonalized = getFromCache(pageUrl, 'personalized');
    const cachedOriginal = getFromCache(pageUrl, 'original');

    if (cachedPersonalized) {
      setHasCachedVersion(true);
      setTranslatedContent(cachedPersonalized.content);
      setTranslationSourceType('personalized');
      currentSourceTypeRef.current = 'personalized';
    } else if (cachedOriginal) {
      setHasCachedVersion(true);
      setTranslatedContent(cachedOriginal.content);
      setTranslationSourceType('original');
      currentSourceTypeRef.current = 'original';
    } else {
      setHasCachedVersion(false);
    }
    setIsContentStale(false);
  }, [pageUrl]);

  // Fetch quota on mount
  useEffect(() => {
    refreshQuota();
  }, []);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (abortControllerRef.current) {
        abortControllerRef.current.abort();
      }
    };
  }, []);

  const refreshQuota = useCallback(async () => {
    try {
      const quota = await apiClient.getQuota();
      setQuotaStatus(quota);
    } catch (err) {
      console.error('[useTranslation] Failed to fetch quota:', err);
    }
  }, [apiClient]);

  const translate = useCallback(
    async (pageContent: string, currentPageUrl: string, sourceType: TranslationSourceType = 'original') => {
      console.log('[useTranslation] translate called:', {
        contentLength: pageContent?.length,
        pageUrl: currentPageUrl,
        sourceType,
      });

      // Update current source type
      currentSourceTypeRef.current = sourceType;

      // Check for cached version first (with matching source type)
      const cached = getFromCache(currentPageUrl, sourceType);
      if (cached) {
        // Use cached version
        setTranslatedContent(cached.content);
        setTranslationSourceType(sourceType);
        setViewMode('translated');
        setState('translated');
        setHasCachedVersion(true);
        return;
      }

      // Cancel any existing request
      if (abortControllerRef.current) {
        abortControllerRef.current.abort();
      }
      abortControllerRef.current = new AbortController();

      setState('loading');
      setError(null);

      try {
        const response = await apiClient.translate({
          page_content: pageContent,
          page_url: currentPageUrl,
        });

        console.log('[useTranslation] API response received:', {
          contentLength: response.translated_content?.length,
          processingTimeMs: response.processing_time_ms,
          sourceType,
        });

        // Save to cache with source type
        saveToCache(
          currentPageUrl,
          response.translated_content,
          response.original_content_hash,
          sourceType,
        );

        setTranslatedContent(response.translated_content);
        setTranslationSourceType(sourceType);
        setQuotaStatus({
          used: response.quota_limit - response.quota_remaining,
          remaining: response.quota_remaining,
          limit: response.quota_limit,
          resets_at: quotaStatus?.resets_at || new Date().toISOString(),
        });
        setViewMode('translated');
        setState('translated');
        setHasCachedVersion(true);
        setIsContentStale(false);
      } catch (err: unknown) {
        console.error('[useTranslation] API call failed:', err);

        if (err instanceof Error && err.name === 'AbortError') {
          // Request was cancelled
          return;
        }

        const apiError = err as { error?: string; code?: string; details?: { used?: number; limit?: number; resets_at?: string } };
        console.error('[useTranslation] Error details:', apiError);

        if (apiError.code === 'QUOTA_EXCEEDED') {
          setState('quota_exceeded');
          if (apiError.details) {
            setQuotaStatus({
              used: apiError.details.used || 5,
              remaining: 0,
              limit: apiError.details.limit || 5,
              resets_at: apiError.details.resets_at || new Date().toISOString(),
            });
          }
          setError('Daily translation limit reached. Try again tomorrow!');
        } else if (apiError.code === 'TIMEOUT') {
          setState('error');
          setError('Translation timed out. Please try again.');
        } else {
          setState('error');
          setError(apiError.error || 'Failed to translate content. Please try again.');
        }
      }
    },
    [apiClient, quotaStatus],
  );

  const toggleView = useCallback(() => {
    console.log('[useTranslation] toggleView called:', {
      currentViewMode: viewMode,
      hasTranslatedContent: !!translatedContent,
    });

    if (viewMode === 'original' && translatedContent) {
      setViewMode('translated');
    } else if (viewMode === 'translated') {
      setViewMode('original');
    }
  }, [viewMode, translatedContent]);

  const loadCachedVersion = useCallback(() => {
    console.log('[useTranslation] loadCachedVersion called:', { pageUrl });

    // Try to load personalized cache first, then original
    const cachedPersonalized = getFromCache(pageUrl, 'personalized');
    const cachedOriginal = getFromCache(pageUrl, 'original');

    const cached = cachedPersonalized || cachedOriginal;
    if (cached) {
      setTranslatedContent(cached.content);
      setTranslationSourceType(cached.sourceType || 'original');
      currentSourceTypeRef.current = cached.sourceType || 'original';
      setViewMode('translated');
      setState('translated');
      console.log('[useTranslation] Loaded cached translation, sourceType:', cached.sourceType);
    }
  }, [pageUrl]);

  const resetTranslation = useCallback(() => {
    console.log('[useTranslation] resetTranslation called - resetting to idle state');
    setState('idle');
    setViewMode('original');
    setTranslatedContent(null);
    setTranslationSourceType(null);
    setError(null);
    currentSourceTypeRef.current = 'original';

    // Check for cached translations for the original content
    const cachedOriginal = getFromCache(pageUrl, 'original');
    if (cachedOriginal) {
      setHasCachedVersion(true);
    } else {
      setHasCachedVersion(false);
    }
  }, [pageUrl]);

  return {
    state,
    viewMode,
    error,
    quotaStatus,
    translatedContent,
    hasCachedVersion,
    isContentStale,
    translationSourceType,
    translate,
    toggleView,
    loadCachedVersion,
    resetTranslation,
  };
}
