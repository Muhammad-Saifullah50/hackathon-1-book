// website/src/components/translation/TranslationBar.tsx
import React, { useEffect, useRef, useCallback } from 'react';
import { useLocation } from '@docusaurus/router';
import { useSafeColorMode } from '../../hooks/useSafeColorMode';
import { useTranslation } from '../../hooks/useTranslation';
import { usePersonalization } from '../../hooks/usePersonalization';
import type { TranslationSourceType } from '../../types/translation';
import {
  Languages,
  Loader2,
  Eye,
  EyeOff,
  AlertCircle,
  RefreshCw,
  Sparkles,
} from 'lucide-react';

const TranslationBar: React.FC = () => {
  const location = useLocation();
  const pageUrl = location.pathname;

  const { isDark } = useSafeColorMode();

  // Get personalization state to know if we're viewing personalized content
  const {
    state: personalizationState,
    viewMode: personalizationViewMode,
  } = usePersonalization(pageUrl);

  const {
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
  } = useTranslation(pageUrl);

  // Determine if we're currently viewing personalized content
  const isViewingPersonalized = personalizationState === 'personalized' && personalizationViewMode === 'personalized';

  // Listen for personalization view change events from PersonalizationBar
  // This is needed because usePersonalization doesn't use Context, so we can't share state directly
  useEffect(() => {
    const handlePersonalizationViewChange = (event: CustomEvent<{ viewMode: string; state: string }>) => {
      const { viewMode: newPersonalizationViewMode, state: newPersonalizationState } = event.detail;

      console.log('[TranslationBar] Received personalization-view-change event:', {
        newPersonalizationViewMode,
        newPersonalizationState,
        currentTranslationState: state,
        currentTranslationViewMode: viewMode,
      });

      // If user is toggling personalization view AND we have an active translation,
      // reset the translation to show the English content (either original or personalized)
      // This ensures that when in Urdu mode, clicking either "Show Original" or "Show Personalized"
      // takes the user back to English content, not Urdu
      if (newPersonalizationState === 'personalized' && state === 'translated') {
        console.log('[TranslationBar] Hiding translation because personalization view is changing');

        // Hide translation container
        const translatedContainer = document.querySelector('#translated-content') as HTMLElement;
        const originalContainer = document.querySelector('#original-content-translation') as HTMLElement;
        if (translatedContainer) {
          translatedContainer.style.display = 'none';
        }
        if (originalContainer) {
          originalContainer.style.display = 'block';
        }

        // Reset the translation state so the bar goes back to idle
        resetTranslation();
      }
    };

    window.addEventListener('personalization-view-change', handlePersonalizationViewChange as EventListener);

    return () => {
      window.removeEventListener('personalization-view-change', handlePersonalizationViewChange as EventListener);
    };
  }, [state, viewMode, resetTranslation]);

  // Get the current source type based on what content is being displayed
  const getCurrentSourceType = useCallback((): TranslationSourceType => {
    return isViewingPersonalized ? 'personalized' : 'original';
  }, [isViewingPersonalized]);

  // Ref to store original content
  const originalContentRef = useRef<string | null>(null);
  // Track if user was viewing personalized content when translation was started
  const wasViewingPersonalizedRef = useRef<boolean>(false);

  // Extract page content from DOM (excluding h1, PersonalizationBar, and TranslationBar)
  // This needs to extract the VISIBLE content, which could be from personalization containers
  const extractPageContent = useCallback((): string | null => {
    const article = document.querySelector('article');
    if (!article) {
      console.warn('[TranslationBar] Could not find article element');
      return null;
    }

    // Check if personalization containers exist - if so, get content from the visible one
    const personalizedContainer = document.querySelector('#personalized-content') as HTMLElement;
    const originalContainer = document.querySelector('#original-content') as HTMLElement;

    // If personalization containers exist, extract from the visible one
    if (personalizedContainer && originalContainer) {
      const isPersonalizedVisible = personalizedContainer.style.display !== 'none';
      const sourceContainer = isPersonalizedVisible ? personalizedContainer : originalContainer;

      console.log('[TranslationBar] Using personalization container, isPersonalizedVisible:', isPersonalizedVisible);

      // Clone and clean up the content
      const clone = sourceContainer.cloneNode(true) as HTMLElement;

      // Remove any nested bar wrappers
      const bars = clone.querySelectorAll('.mt-4.mb-6, .translation-bar-wrapper');
      bars.forEach(bar => bar.remove());

      const content = clone.innerHTML;
      console.log('[TranslationBar] Extracted content from personalization container, length:', content.length);
      return content;
    }

    // Fallback: Look for the markdown content container
    const markdownContent = article.querySelector('.markdown') ||
                           article.querySelector('[class*="mdxPageWrapper"]') ||
                           article.querySelector('.theme-doc-markdown');

    if (!markdownContent) {
      console.warn('[TranslationBar] Could not find markdown container');
      return null;
    }

    // Clone the container to avoid modifying the original
    const clone = markdownContent.cloneNode(true) as HTMLElement;

    // Remove h1 heading from the clone
    const h1 = clone.querySelector('h1');
    if (h1) h1.remove();

    // Remove PersonalizationBar and TranslationBar wrappers from the clone
    const bars = clone.querySelectorAll('.mt-4.mb-6, .translation-bar-wrapper');
    bars.forEach(bar => bar.remove());

    // Also remove any translation containers from the clone
    const translationContainers = clone.querySelectorAll('#original-content-translation, #translated-content');
    translationContainers.forEach(container => container.remove());

    const content = clone.innerHTML;
    console.log('[TranslationBar] Extracted content from markdown, length:', content.length);
    return content;
  }, []);

  // Reset original content ref on page navigation
  useEffect(() => {
    originalContentRef.current = null;
  }, [pageUrl]);

  // Apply translated content to the page using dual-container approach
  // This works with or without personalization containers
  const applyContent = useCallback((content: string, isTranslated: boolean) => {
    console.log('[TranslationBar] applyContent called:', { isTranslated, contentLength: content?.length });

    const article = document.querySelector('article');
    if (!article) {
      console.warn('[TranslationBar] Could not find article element');
      return;
    }

    // Check if personalization containers exist
    const personalizedContainer = document.querySelector('#personalized-content') as HTMLElement;
    const originalPersonalizationContainer = document.querySelector('#original-content') as HTMLElement;
    const isPersonalizationActive = personalizedContainer && originalPersonalizationContainer;
    // Use the tracked state for what the user was viewing when translation started
    const shouldRestorePersonalized = wasViewingPersonalizedRef.current;

    console.log('[TranslationBar] Personalization state:', { isPersonalizationActive, shouldRestorePersonalized });

    // Find the content containers - they might already exist
    let originalContainer = document.querySelector('#original-content-translation') as HTMLElement;
    let translatedContainer = document.querySelector('#translated-content') as HTMLElement;

    // If containers already exist, just toggle visibility
    if (originalContainer && translatedContainer) {
      // Update translated content if provided
      if (isTranslated && content) {
        translatedContainer.innerHTML = content;
      }

      // Toggle visibility
      if (isTranslated) {
        originalContainer.style.display = 'none';
        translatedContainer.style.display = 'block';
        // Also hide personalization containers when showing translation
        if (isPersonalizationActive) {
          personalizedContainer.style.display = 'none';
          originalPersonalizationContainer.style.display = 'none';
        }
        console.log('[TranslationBar] Showing translated content');
      } else {
        originalContainer.style.display = 'none'; // Hide translation's original container
        translatedContainer.style.display = 'none';
        // Show the appropriate personalization container based on what user was viewing before translation
        if (isPersonalizationActive) {
          if (shouldRestorePersonalized) {
            personalizedContainer.style.display = 'block';
            originalPersonalizationContainer.style.display = 'none';
          } else {
            personalizedContainer.style.display = 'none';
            originalPersonalizationContainer.style.display = 'block';
          }
        } else {
          // No personalization - show translation's original container
          originalContainer.style.display = 'block';
        }
        console.log('[TranslationBar] Showing original content, restorePersonalized:', shouldRestorePersonalized);
      }
      return;
    }

    // First time setup: need to create containers
    const markdownContent = article.querySelector('.markdown, .theme-doc-markdown') as HTMLElement;

    if (!markdownContent) {
      console.warn('[TranslationBar] Could not find markdown content');
      return;
    }

    // Determine source content - if personalization is active, use the appropriate container's content
    let sourceHTML: string;
    if (isPersonalizationActive) {
      const sourceContainer = shouldRestorePersonalized ? personalizedContainer : originalPersonalizationContainer;
      sourceHTML = sourceContainer.innerHTML;
      console.log('[TranslationBar] Using personalization container content, shouldRestorePersonalized:', shouldRestorePersonalized);
    } else {
      // Find elements to preserve (like h1 and bar wrappers)
      const h1Element = markdownContent.querySelector('h1');
      const barWrappers = markdownContent.querySelectorAll('.mt-4.mb-6, .translation-bar-wrapper');

      // Get all child nodes except h1 and bar wrappers
      const contentNodes: Node[] = [];
      markdownContent.childNodes.forEach((node) => {
        // Skip the h1 and bar wrappers
        if (node === h1Element) return;
        if (node instanceof Element) {
          if (node.classList.contains('mt-4') || node.classList.contains('translation-bar-wrapper')) return;
          if (node.contains(h1Element)) return;
          let shouldSkip = false;
          barWrappers.forEach(bar => {
            if (node.contains(bar) || node === bar) shouldSkip = true;
          });
          if (shouldSkip) return;
        }
        contentNodes.push(node.cloneNode(true));
      });

      // Create a temporary container to get innerHTML of content only
      const tempContainer = document.createElement('div');
      contentNodes.forEach((node) => tempContainer.appendChild(node));
      sourceHTML = tempContainer.innerHTML;
    }

    // Create containers
    originalContainer = document.createElement('div');
    originalContainer.id = 'original-content-translation';
    originalContainer.innerHTML = sourceHTML;
    originalContainer.style.display = 'none'; // Start hidden

    translatedContainer = document.createElement('div');
    translatedContainer.id = 'translated-content';
    translatedContainer.className = '[direction:rtl] text-right';
    translatedContainer.style.display = 'none';

    // If personalization is NOT active, we need to remove content nodes and set up containers
    if (!isPersonalizationActive) {
      const h1Element = markdownContent.querySelector('h1');
      const barWrappers = markdownContent.querySelectorAll('.mt-4.mb-6, .translation-bar-wrapper');

      // Remove only the content nodes (keep h1 and bar wrappers)
      const nodesToRemove: Node[] = [];
      markdownContent.childNodes.forEach((node) => {
        if (node === h1Element) return;
        if (node instanceof Element) {
          if (node.classList.contains('mt-4') || node.classList.contains('translation-bar-wrapper')) return;
          if (node.contains(h1Element)) return;
          let shouldSkip = false;
          barWrappers.forEach(bar => {
            if (node.contains(bar) || node === bar) shouldSkip = true;
          });
          if (shouldSkip) return;
          if (node.querySelector('h1') || node.querySelector('.mt-4.mb-6')) return;
        }
        nodesToRemove.push(node);
      });
      nodesToRemove.forEach((node) => node.parentNode?.removeChild(node));
    }

    // Append our containers at the end
    markdownContent.appendChild(originalContainer);
    markdownContent.appendChild(translatedContainer);

    console.log('[TranslationBar] Created content containers');

    // Update translated content if provided
    if (isTranslated && content) {
      translatedContainer.innerHTML = content;
    }

    // Toggle visibility
    if (isTranslated) {
      originalContainer.style.display = 'none';
      translatedContainer.style.display = 'block';
      // Hide personalization containers when showing translation
      if (isPersonalizationActive) {
        personalizedContainer.style.display = 'none';
        originalPersonalizationContainer.style.display = 'none';
      }
      console.log('[TranslationBar] Showing translated content');
    } else {
      originalContainer.style.display = 'none';
      translatedContainer.style.display = 'none';
      // Show appropriate content based on what user was viewing before translation
      if (isPersonalizationActive) {
        if (shouldRestorePersonalized) {
          personalizedContainer.style.display = 'block';
          originalPersonalizationContainer.style.display = 'none';
        } else {
          personalizedContainer.style.display = 'none';
          originalPersonalizationContainer.style.display = 'block';
        }
      } else {
        originalContainer.style.display = 'block';
      }
      console.log('[TranslationBar] Showing original content, restorePersonalized:', shouldRestorePersonalized);
    }
  }, []);

  // Handle content display based on view mode
  useEffect(() => {
    console.log('[TranslationBar] View mode effect triggered:', {
      viewMode,
      hasTranslatedContent: !!translatedContent,
      state,
    });

    // When state resets to idle, make sure translated container is hidden
    if (state === 'idle') {
      const translatedContainer = document.querySelector('#translated-content') as HTMLElement;
      const originalContainer = document.querySelector('#original-content-translation') as HTMLElement;
      if (translatedContainer) {
        translatedContainer.style.display = 'none';
      }
      if (originalContainer) {
        originalContainer.style.display = 'block';
      }
      return;
    }

    // Only apply content changes when we're in translated state
    if (state !== 'translated') {
      return;
    }

    if (viewMode === 'translated' && translatedContent) {
      applyContent(translatedContent, true);
    } else if (viewMode === 'original') {
      applyContent('', false);
    }
  }, [viewMode, translatedContent, applyContent, state]);

  const handleTranslateClick = async () => {
    const sourceType = getCurrentSourceType();
    console.log('[TranslationBar] Translate button clicked, sourceType:', sourceType);

    // Track if we're translating from personalized content
    wasViewingPersonalizedRef.current = isViewingPersonalized;

    // Extract content (could be original or personalized depending on what's displayed)
    const content = extractPageContent();
    if (!content) {
      console.error('[TranslationBar] Failed to extract page content');
      return;
    }
    originalContentRef.current = content;

    await translate(content, pageUrl, sourceType);
  };

  const handleToggleView = () => {
    console.log('[TranslationBar] handleToggleView called');
    // Store original content before first toggle if needed
    if (!originalContentRef.current) {
      const content = extractPageContent();
      if (content) {
        originalContentRef.current = content;
      }
    }
    toggleView();
  };

  const handleLoadCached = () => {
    // Track if we're loading cached from personalized content view
    wasViewingPersonalizedRef.current = isViewingPersonalized;

    // Store original content before loading cached
    if (!originalContentRef.current) {
      const content = extractPageContent();
      if (content) {
        originalContentRef.current = content;
      }
    }
    loadCachedVersion();
  };

  // Error state
  if (state === 'error' || state === 'quota_exceeded') {
    return (
      <div className="translation-bar-wrapper mt-2 mb-4">
        <div
          className={`flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4 p-4 rounded-xl ${
            isDark
              ? 'bg-gradient-to-r from-red-500/10 to-orange-500/10 border border-red-500/20'
              : 'bg-gradient-to-r from-red-50 to-orange-50 border border-red-200'
          }`}
        >
          <div className="flex items-start gap-3">
            <div
              className={`flex items-center justify-center w-10 h-10 rounded-lg flex-shrink-0 ${
                isDark ? 'bg-red-500/20' : 'bg-red-100'
              }`}
            >
              <AlertCircle
                className={`w-5 h-5 ${isDark ? 'text-red-400' : 'text-red-600'}`}
              />
            </div>
            <div>
              <p
                className={`font-semibold ${
                  isDark ? 'text-slate-200' : 'text-slate-800'
                }`}
              >
                {state === 'quota_exceeded' ? 'Daily limit reached' : 'Translation failed'}
              </p>
              <p
                className={`text-sm ${isDark ? 'text-slate-400' : 'text-slate-600'}`}
              >
                {error || 'Something went wrong. Please try again.'}
              </p>
            </div>
          </div>
          {state !== 'quota_exceeded' && (
            <button
              onClick={handleTranslateClick}
              className={`flex items-center gap-2 px-4 py-2 rounded-lg text-sm font-semibold transition-all duration-200 ${
                isDark
                  ? 'bg-red-500 hover:bg-red-400 text-white'
                  : 'bg-red-600 hover:bg-red-500 text-white'
              }`}
            >
              <RefreshCw className="w-4 h-4" />
              Retry
            </button>
          )}
        </div>
      </div>
    );
  }

  // Loading state
  if (state === 'loading') {
    return (
      <div className="translation-bar-wrapper mt-2 mb-4">
        <div
          className={`flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4 p-4 rounded-xl ${
            isDark
              ? 'bg-gradient-to-r from-blue-500/10 to-cyan-500/10 border border-blue-500/20'
              : 'bg-gradient-to-r from-blue-50 to-cyan-50 border border-blue-200'
          }`}
        >
          <div className="flex items-start gap-3">
            <div
              className={`flex items-center justify-center w-10 h-10 rounded-lg flex-shrink-0 ${
                isDark ? 'bg-blue-500/20' : 'bg-blue-100'
              }`}
            >
              <Loader2
                className={`w-5 h-5 animate-spin ${isDark ? 'text-blue-400' : 'text-blue-600'}`}
              />
            </div>
            <div>
              <p
                className={`font-semibold ${
                  isDark ? 'text-slate-200' : 'text-slate-800'
                }`}
              >
                Translating to Urdu...
              </p>
              <p
                className={`text-sm ${isDark ? 'text-slate-400' : 'text-slate-600'}`}
              >
                Converting content to{' '}
                <span className={`font-medium ${isDark ? 'text-blue-400' : 'text-blue-600'}`}>
                  اردو
                </span>
              </p>
            </div>
          </div>
          <div
            className={`px-4 py-2 rounded-lg text-sm font-semibold ${
              isDark ? 'bg-blue-500/20 text-blue-400' : 'bg-blue-100 text-blue-700'
            }`}
          >
            Processing...
          </div>
        </div>
      </div>
    );
  }

  // Translated state - show toggle
  if (state === 'translated') {
    // Determine the label based on source type
    const isFromPersonalized = translationSourceType === 'personalized';
    const originalLabel = isFromPersonalized ? 'Show Personalized (English)' : 'Show Original (English)';
    const translatedLabel = isFromPersonalized ? 'Urdu (Personalized)' : 'Urdu (Original)';

    return (
      <div className="translation-bar-wrapper mt-2 mb-4">
        <div
          className={`flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4 p-4 rounded-xl ${
            isDark
              ? 'bg-gradient-to-r from-blue-500/10 to-cyan-500/10 border border-blue-500/20'
              : 'bg-gradient-to-r from-blue-50 to-cyan-50 border border-blue-200'
          }`}
        >
          <div className="flex items-start gap-3">
            <div
              className={`flex items-center justify-center w-10 h-10 rounded-lg flex-shrink-0 ${
                isDark ? 'bg-blue-500/20' : 'bg-blue-100'
              }`}
            >
              {isFromPersonalized ? (
                <Sparkles className={`w-5 h-5 ${isDark ? 'text-blue-400' : 'text-blue-600'}`} />
              ) : (
                <Languages className={`w-5 h-5 ${isDark ? 'text-blue-400' : 'text-blue-600'}`} />
              )}
            </div>
            <div>
              <p
                className={`font-semibold ${
                  isDark ? 'text-slate-200' : 'text-slate-800'
                }`}
              >
                {viewMode === 'translated'
                  ? `Viewing ${translatedLabel}`
                  : `Viewing ${isFromPersonalized ? 'Personalized' : 'Original'} (English)`}
              </p>
              <p
                className={`text-sm ${isDark ? 'text-slate-400' : 'text-slate-600'}`}
              >
                {viewMode === 'translated'
                  ? `اردو میں مواد ${isFromPersonalized ? '(ذاتی نوعیت کا)' : ''}`
                  : isFromPersonalized ? 'Personalized English content' : 'Original English content'}
                {quotaStatus && (
                  <span className="ml-2">
                    ({quotaStatus.remaining}/{quotaStatus.limit} translations left today)
                  </span>
                )}
              </p>
            </div>
          </div>
          <button
            onClick={handleToggleView}
            className={`flex items-center gap-2 px-4 py-2 rounded-lg text-sm font-semibold transition-all duration-200 ${
              isDark
                ? 'bg-slate-600 hover:bg-slate-500 text-white'
                : 'bg-slate-200 hover:bg-slate-300 text-slate-800'
            }`}
          >
            {viewMode === 'translated' ? (
              <>
                <EyeOff className="w-4 h-4" />
                {originalLabel}
              </>
            ) : (
              <>
                <Eye className="w-4 h-4" />
                Show اردو
              </>
            )}
          </button>
        </div>
      </div>
    );
  }

  // Cached version available
  if (hasCachedVersion) {
    return (
      <div className="translation-bar-wrapper mt-2 mb-4">
        <div
          className={`flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4 p-4 rounded-xl ${
            isDark
              ? 'bg-gradient-to-r from-blue-500/10 to-cyan-500/10 border border-blue-500/20'
              : 'bg-gradient-to-r from-blue-50 to-cyan-50 border border-blue-200'
          }`}
        >
          <div className="flex items-start gap-3">
            <div
              className={`flex items-center justify-center w-10 h-10 rounded-lg flex-shrink-0 ${
                isDark ? 'bg-blue-500/20' : 'bg-blue-100'
              }`}
            >
              <Languages
                className={`w-5 h-5 ${isDark ? 'text-blue-400' : 'text-blue-600'}`}
              />
            </div>
            <div>
              <p
                className={`font-semibold ${
                  isDark ? 'text-slate-200' : 'text-slate-800'
                }`}
              >
                Urdu translation available
              </p>
              <p
                className={`text-sm ${isDark ? 'text-slate-400' : 'text-slate-600'}`}
              >
                {isContentStale
                  ? 'Content has changed - re-translate for latest version'
                  : 'You previously translated this page'}
              </p>
            </div>
          </div>
          <div className="flex gap-2">
            <button
              onClick={handleLoadCached}
              className={`flex items-center gap-2 px-4 py-2 rounded-lg text-sm font-semibold transition-all duration-200 ${
                isDark
                  ? 'bg-blue-500 hover:bg-blue-400 text-white shadow-lg shadow-blue-500/20'
                  : 'bg-blue-600 hover:bg-blue-500 text-white shadow-lg shadow-blue-600/20'
              }`}
            >
              <Eye className="w-4 h-4" />
              View اردو
            </button>
            {isContentStale && (
              <button
                onClick={handleTranslateClick}
                className={`flex items-center gap-2 px-4 py-2 rounded-lg text-sm font-semibold transition-all duration-200 ${
                  isDark
                    ? 'bg-amber-500 hover:bg-amber-400 text-white'
                    : 'bg-amber-600 hover:bg-amber-500 text-white'
                }`}
              >
                <RefreshCw className="w-4 h-4" />
                Re-translate
              </button>
            )}
          </div>
        </div>
      </div>
    );
  }

  // Default idle state - offer to translate
  // Show different message if translating personalized content
  const willTranslatePersonalized = isViewingPersonalized;

  return (
    <div className="translation-bar-wrapper mt-2 mb-4">
      <div
        className={`flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4 p-4 rounded-xl ${
          isDark
            ? 'bg-gradient-to-r from-blue-500/10 to-cyan-500/10 border border-blue-500/20'
            : 'bg-gradient-to-r from-blue-50 to-cyan-50 border border-blue-200'
        }`}
      >
        <div className="flex items-start gap-3">
          <div
            className={`flex items-center justify-center w-10 h-10 rounded-lg flex-shrink-0 ${
              isDark ? 'bg-blue-500/20' : 'bg-blue-100'
            }`}
          >
            {willTranslatePersonalized ? (
              <Sparkles className={`w-5 h-5 ${isDark ? 'text-blue-400' : 'text-blue-600'}`} />
            ) : (
              <Languages className={`w-5 h-5 ${isDark ? 'text-blue-400' : 'text-blue-600'}`} />
            )}
          </div>
          <div>
            <p
              className={`font-semibold ${
                isDark ? 'text-slate-200' : 'text-slate-800'
              }`}
            >
              Translate to Urdu?
            </p>
            <p
              className={`text-sm ${isDark ? 'text-slate-400' : 'text-slate-600'}`}
            >
              {willTranslatePersonalized ? (
                <>
                  Translate your{' '}
                  <span className={`font-medium ${isDark ? 'text-emerald-400' : 'text-emerald-600'}`}>
                    personalized
                  </span>{' '}
                  content to{' '}
                  <span className={`font-medium ${isDark ? 'text-blue-400' : 'text-blue-600'}`}>
                    اردو
                  </span>
                </>
              ) : (
                <>
                  Read this page in{' '}
                  <span className={`font-medium ${isDark ? 'text-blue-400' : 'text-blue-600'}`}>
                    اردو
                  </span>
                </>
              )}
              {quotaStatus && (
                <span className="ml-1">
                  ({quotaStatus.remaining}/{quotaStatus.limit} left today)
                </span>
              )}
            </p>
          </div>
        </div>
        <button
          onClick={handleTranslateClick}
          disabled={quotaStatus?.remaining === 0}
          className={`flex items-center gap-2 px-4 py-2 rounded-lg text-sm font-semibold transition-all duration-200 ${
            quotaStatus?.remaining === 0
              ? isDark
                ? 'bg-slate-600 text-slate-400 cursor-not-allowed'
                : 'bg-slate-300 text-slate-500 cursor-not-allowed'
              : isDark
                ? 'bg-blue-500 hover:bg-blue-400 text-white shadow-lg shadow-blue-500/20'
                : 'bg-blue-600 hover:bg-blue-500 text-white shadow-lg shadow-blue-600/20'
          } focus:outline-none focus:ring-2 focus:ring-blue-500 focus:ring-offset-2 ${
            isDark ? 'focus:ring-offset-slate-900' : 'focus:ring-offset-white'
          }`}
        >
          <Languages className="w-4 h-4" />
          Translate to اردو
        </button>
      </div>
    </div>
  );
};

export default TranslationBar;
