// website/src/components/profile/PersonalizationBar.tsx
import React, { useEffect, useRef, useCallback } from 'react';
import { useLocation } from '@docusaurus/router';
import { useProfile } from '../../hooks/useProfile.tsx';
import { useAuth } from '../../hooks/useAuth';
import { useSafeColorMode } from '../../hooks/useSafeColorMode';
import { usePersonalization } from '../../hooks/usePersonalization';
import {
  Sparkles,
  Wand2,
  Loader2,
  RotateCcw,
  Eye,
  EyeOff,
  AlertCircle,
  RefreshCw,
  LogIn,
  UserCircle,
} from 'lucide-react';

const PersonalizationBar: React.FC = () => {
  const location = useLocation();
  const pageUrl = location.pathname;

  const { user, loading: loadingAuth } = useAuth();
  const { profile, loadingProfile } = useProfile();
  const { isDark } = useSafeColorMode();

  const {
    state,
    viewMode,
    error,
    quotaStatus,
    personalizedContent,
    hasCachedVersion,
    isProfileStale,
    personalize,
    toggleView,
    loadCachedVersion,
  } = usePersonalization(pageUrl);

  // Ref to store original content
  const originalContentRef = useRef<string | null>(null);
  const articleRef = useRef<HTMLElement | null>(null);

  // Extract page content from DOM (excluding h1 and PersonalizationBar)
  const extractPageContent = useCallback((): string | null => {
    const article = document.querySelector('article');
    if (!article) {
      console.warn('[PersonalizationBar] Could not find article element');
      return null;
    }

    // Look for the markdown content container
    const markdownContent = article.querySelector('.markdown') ||
                           article.querySelector('[class*="mdxPageWrapper"]') ||
                           article.querySelector('.theme-doc-markdown');

    if (!markdownContent) {
      console.warn('[PersonalizationBar] Could not find markdown container');
      return null;
    }

    articleRef.current = markdownContent as HTMLElement;

    // Clone the container to avoid modifying the original
    const clone = markdownContent.cloneNode(true) as HTMLElement;

    // Remove h1 heading from the clone
    const h1 = clone.querySelector('h1');
    if (h1) h1.remove();

    // Remove PersonalizationBar wrapper from the clone
    const personalizationBar = clone.querySelector('.mt-4.mb-6');
    if (personalizationBar) personalizationBar.remove();

    const content = clone.innerHTML;
    console.log('[PersonalizationBar] Extracted content (excluding h1 and bar), length:', content.length);
    return content;
  }, []);

  // Initialize articleRef on mount and page navigation
  // Note: We don't capture original content here - only when user clicks personalize
  useEffect(() => {
    const article = document.querySelector('article');
    if (article) {
      // Try to find the markdown content container
      const markdownContent = article.querySelector('.markdown') ||
                             article.querySelector('[class*="mdxPageWrapper"]') ||
                             article.querySelector('.theme-doc-markdown');
      articleRef.current = (markdownContent as HTMLElement) || article;
    }
    // Reset original content ref on page change
    originalContentRef.current = null;
  }, [pageUrl]);

  // Apply personalized content to the page
  // Uses a dual-container approach: hides original, shows personalized (or vice versa)
  const applyContent = useCallback((content: string, isPersonalized: boolean) => {
    console.log('[PersonalizationBar] applyContent called:', { isPersonalized, contentLength: content?.length });

    const article = document.querySelector('article');
    if (!article) {
      console.warn('[PersonalizationBar] Could not find article element');
      return;
    }

    // Find the content containers - they might already exist
    let originalContainer = document.querySelector('#original-content') as HTMLElement;
    let personalizedContainer = document.querySelector('#personalized-content') as HTMLElement;

    console.log('[PersonalizationBar] Existing containers:', {
      hasOriginal: !!originalContainer,
      hasPersonalized: !!personalizedContainer
    });

    // If containers already exist, just toggle visibility
    if (originalContainer && personalizedContainer) {
      // Update personalized content if provided
      if (isPersonalized && content) {
        personalizedContainer.innerHTML = content;
      }

      // Toggle visibility
      if (isPersonalized) {
        originalContainer.style.display = 'none';
        personalizedContainer.style.display = 'block';
        console.log('[PersonalizationBar] Showing personalized content');
      } else {
        originalContainer.style.display = 'block';
        personalizedContainer.style.display = 'none';
        console.log('[PersonalizationBar] Showing original content');
      }
      return;
    }

    // First time setup: need to create containers
    const markdownContent = article.querySelector('.markdown, .theme-doc-markdown') as HTMLElement;

    if (!markdownContent) {
      console.warn('[PersonalizationBar] Could not find markdown content');
      return;
    }

    // Find elements to preserve (like h1 and PersonalizationBar wrapper)
    // These are typically at the start of the markdown content
    const h1Element = markdownContent.querySelector('h1');
    const personalizationBarWrapper = markdownContent.querySelector('.mt-4.mb-6');

    // Get all child nodes except h1 and personalization bar
    const contentNodes: Node[] = [];
    markdownContent.childNodes.forEach((node) => {
      // Skip the h1 and its wrapper, and the personalization bar wrapper
      if (node === h1Element || node === personalizationBarWrapper) {
        return;
      }
      if (node instanceof Element) {
        if (node.contains(h1Element) || node.contains(personalizationBarWrapper)) {
          return;
        }
      }
      contentNodes.push(node.cloneNode(true));
    });

    // Create a temporary container to get innerHTML of content only
    const tempContainer = document.createElement('div');
    contentNodes.forEach((node) => tempContainer.appendChild(node));
    const originalHTML = tempContainer.innerHTML;

    // Create containers
    originalContainer = document.createElement('div');
    originalContainer.id = 'original-content';
    originalContainer.innerHTML = originalHTML;

    personalizedContainer = document.createElement('div');
    personalizedContainer.id = 'personalized-content';
    personalizedContainer.style.display = 'none';

    // Remove only the content nodes (keep h1 and personalization bar)
    const nodesToRemove: Node[] = [];
    markdownContent.childNodes.forEach((node) => {
      if (node === h1Element || node === personalizationBarWrapper) {
        return;
      }
      if (node instanceof Element) {
        if (node.contains(h1Element) || node.contains(personalizationBarWrapper)) {
          return;
        }
        // Also skip if this element contains h1 or personalization bar
        if (node.querySelector('h1') || node.querySelector('.mt-4.mb-6')) {
          return;
        }
      }
      nodesToRemove.push(node);
    });
    nodesToRemove.forEach((node) => node.parentNode?.removeChild(node));

    // Append our containers at the end
    markdownContent.appendChild(originalContainer);
    markdownContent.appendChild(personalizedContainer);

    console.log('[PersonalizationBar] Created content containers');

    // Update personalized content if provided
    if (isPersonalized && content) {
      personalizedContainer.innerHTML = content;
    }

    // Toggle visibility
    if (isPersonalized) {
      originalContainer.style.display = 'none';
      personalizedContainer.style.display = 'block';
      console.log('[PersonalizationBar] Showing personalized content');
    } else {
      originalContainer.style.display = 'block';
      personalizedContainer.style.display = 'none';
      console.log('[PersonalizationBar] Showing original content');
    }
  }, []);

  // Handle content display based on view mode
  // Only apply content when we have personalized content AND user explicitly changes view mode
  useEffect(() => {
    console.log('[PersonalizationBar] View mode effect triggered:', {
      viewMode,
      hasPersonalizedContent: !!personalizedContent,
      hasOriginalContent: !!originalContentRef.current,
      hasArticleRef: !!articleRef.current,
      state,
    });

    // Only apply content changes when we're in personalized state
    // This prevents overwriting content during initial load
    if (state !== 'personalized') {
      console.log('[PersonalizationBar] Skipping content apply - not in personalized state');
      return;
    }

    if (viewMode === 'personalized' && personalizedContent) {
      applyContent(personalizedContent, true);
    } else if (viewMode === 'original') {
      applyContent('', false);
    }
  }, [viewMode, personalizedContent, applyContent, state]);

  const handlePersonalizeClick = async () => {
    console.log('[PersonalizationBar] Personalize button clicked');
    console.log('[PersonalizationBar] Current state:', { state, pageUrl, hasProfile: !!profile });

    // Extract and store original content if not already done
    if (!originalContentRef.current) {
      const content = extractPageContent();
      console.log('[PersonalizationBar] Extracted content length:', content?.length);
      if (!content) {
        console.error('[PersonalizationBar] Failed to extract page content');
        return;
      }
      originalContentRef.current = content;
    }

    console.log('[PersonalizationBar] Calling personalize with content length:', originalContentRef.current.length);
    await personalize(originalContentRef.current, pageUrl);
    console.log('[PersonalizationBar] Personalize call completed, new state:', state);
  };

  const handleToggleView = () => {
    console.log('[PersonalizationBar] handleToggleView called');
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
    // Store original content before loading cached
    if (!originalContentRef.current) {
      const content = extractPageContent();
      if (content) {
        originalContentRef.current = content;
      }
    }
    loadCachedVersion();
  };

  // Debug: Log which render path we're taking
  console.log('[PersonalizationBar] Render state:', {
    loadingAuth,
    loadingProfile,
    hasUser: !!user,
    hasProfile: !!profile,
    state,
    hasCachedVersion,
    quotaRemaining: quotaStatus?.remaining,
  });

  // Loading auth/profile state
  if (loadingAuth || loadingProfile) {
    console.log('[PersonalizationBar] Returning null - loading');
    return null;
  }

  // Not logged in - show login prompt
  if (!user) {
    console.log('[PersonalizationBar] Showing login prompt');
    return (
      <div
        className={`flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4 p-4 rounded-xl mb-6 ${
          isDark
            ? 'bg-gradient-to-r from-slate-500/10 to-slate-600/10 border border-slate-500/20'
            : 'bg-gradient-to-r from-slate-50 to-slate-100 border border-slate-200'
        }`}
      >
        <div className="flex items-start gap-3">
          <div
            className={`flex items-center justify-center w-10 h-10 rounded-lg flex-shrink-0 ${
              isDark ? 'bg-slate-500/20' : 'bg-slate-100'
            }`}
          >
            <Sparkles
              className={`w-5 h-5 ${isDark ? 'text-slate-400' : 'text-slate-600'}`}
            />
          </div>
          <div>
            <p
              className={`font-semibold ${
                isDark ? 'text-slate-200' : 'text-slate-800'
              }`}
            >
              Personalize this page
            </p>
            <p
              className={`text-sm ${isDark ? 'text-slate-400' : 'text-slate-600'}`}
            >
              Log in to tailor content to your learning style
            </p>
          </div>
        </div>
        <a
          href="/login"
          className={`flex items-center gap-2 px-4 py-2 rounded-lg text-sm font-semibold transition-all duration-200 ${
            isDark
              ? 'bg-slate-600 hover:bg-slate-500 text-white'
              : 'bg-slate-700 hover:bg-slate-600 text-white'
          }`}
        >
          <LogIn className="w-4 h-4" />
          Log in
        </a>
      </div>
    );
  }

  // Logged in but no profile - show complete profile prompt
  if (!profile) {
    return (
      <div
        className={`flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4 p-4 rounded-xl mb-6 ${
          isDark
            ? 'bg-gradient-to-r from-amber-500/10 to-orange-500/10 border border-amber-500/20'
            : 'bg-gradient-to-r from-amber-50 to-orange-50 border border-amber-200'
        }`}
      >
        <div className="flex items-start gap-3">
          <div
            className={`flex items-center justify-center w-10 h-10 rounded-lg flex-shrink-0 ${
              isDark ? 'bg-amber-500/20' : 'bg-amber-100'
            }`}
          >
            <UserCircle
              className={`w-5 h-5 ${isDark ? 'text-amber-400' : 'text-amber-600'}`}
            />
          </div>
          <div>
            <p
              className={`font-semibold ${
                isDark ? 'text-slate-200' : 'text-slate-800'
              }`}
            >
              Complete your profile
            </p>
            <p
              className={`text-sm ${isDark ? 'text-slate-400' : 'text-slate-600'}`}
            >
              Set up your learning preferences to unlock personalization
            </p>
          </div>
        </div>
        <a
          href="/signup-wizard"
          className={`flex items-center gap-2 px-4 py-2 rounded-lg text-sm font-semibold transition-all duration-200 ${
            isDark
              ? 'bg-amber-500 hover:bg-amber-400 text-white'
              : 'bg-amber-600 hover:bg-amber-500 text-white'
          }`}
        >
          <UserCircle className="w-4 h-4" />
          Complete Profile
        </a>
      </div>
    );
  }

  // Error state
  if (state === 'error' || state === 'quota_exceeded') {
    return (
      <div
        className={`flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4 p-4 rounded-xl mb-6 ${
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
              {state === 'quota_exceeded' ? 'Daily limit reached' : 'Personalization failed'}
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
            onClick={handlePersonalizeClick}
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
    );
  }

  // Loading state
  if (state === 'loading') {
    return (
      <div
        className={`flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4 p-4 rounded-xl mb-6 ${
          isDark
            ? 'bg-gradient-to-r from-emerald-500/10 to-amber-500/10 border border-emerald-500/20'
            : 'bg-gradient-to-r from-emerald-50 to-amber-50 border border-emerald-200'
        }`}
      >
        <div className="flex items-start gap-3">
          <div
            className={`flex items-center justify-center w-10 h-10 rounded-lg flex-shrink-0 ${
              isDark ? 'bg-emerald-500/20' : 'bg-emerald-100'
            }`}
          >
            <Loader2
              className={`w-5 h-5 animate-spin ${isDark ? 'text-emerald-400' : 'text-emerald-600'}`}
            />
          </div>
          <div>
            <p
              className={`font-semibold ${
                isDark ? 'text-slate-200' : 'text-slate-800'
              }`}
            >
              Personalizing content...
            </p>
            <p
              className={`text-sm ${isDark ? 'text-slate-400' : 'text-slate-600'}`}
            >
              Adapting content to your{' '}
              <span className={`font-medium ${isDark ? 'text-emerald-400' : 'text-emerald-600'}`}>
                {profile.tech_background?.replace('_', ' ') || 'learning'}{' '}
              </span>
              background
            </p>
          </div>
        </div>
        <div
          className={`px-4 py-2 rounded-lg text-sm font-semibold ${
            isDark ? 'bg-emerald-500/20 text-emerald-400' : 'bg-emerald-100 text-emerald-700'
          }`}
        >
          Processing...
        </div>
      </div>
    );
  }

  // Personalized state - show toggle
  if (state === 'personalized') {
    console.log('[PersonalizationBar] Rendering personalized state UI with toggle button, viewMode:', viewMode);
    return (
      <div
        className={`flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4 p-4 rounded-xl mb-6 ${
          isDark
            ? 'bg-gradient-to-r from-emerald-500/10 to-cyan-500/10 border border-emerald-500/20'
            : 'bg-gradient-to-r from-emerald-50 to-cyan-50 border border-emerald-200'
        }`}
      >
        <div className="flex items-start gap-3">
          <div
            className={`flex items-center justify-center w-10 h-10 rounded-lg flex-shrink-0 ${
              isDark ? 'bg-emerald-500/20' : 'bg-emerald-100'
            }`}
          >
            <Sparkles
              className={`w-5 h-5 ${isDark ? 'text-emerald-400' : 'text-emerald-600'}`}
            />
          </div>
          <div>
            <p
              className={`font-semibold ${
                isDark ? 'text-slate-200' : 'text-slate-800'
              }`}
            >
              {viewMode === 'personalized' ? 'Viewing personalized content' : 'Viewing original content'}
            </p>
            <p
              className={`text-sm ${isDark ? 'text-slate-400' : 'text-slate-600'}`}
            >
              {viewMode === 'personalized'
                ? 'Content adapted to your learning profile'
                : 'Original documentation content'}
              {quotaStatus && (
                <span className="ml-2">
                  ({quotaStatus.remaining}/{quotaStatus.limit} personalizations left today)
                </span>
              )}
            </p>
          </div>
        </div>
        <div className="flex gap-2">
          <button
            onClick={handleToggleView}
            className={`flex items-center gap-2 px-4 py-2 rounded-lg text-sm font-semibold transition-all duration-200 ${
              isDark
                ? 'bg-slate-600 hover:bg-slate-500 text-white'
                : 'bg-slate-200 hover:bg-slate-300 text-slate-800'
            }`}
          >
            {viewMode === 'personalized' ? (
              <>
                <EyeOff className="w-4 h-4" />
                Show Original
              </>
            ) : (
              <>
                <Eye className="w-4 h-4" />
                Show Personalized
              </>
            )}
          </button>
          {isProfileStale && (
            <button
              onClick={handlePersonalizeClick}
              className={`flex items-center gap-2 px-4 py-2 rounded-lg text-sm font-semibold transition-all duration-200 ${
                isDark
                  ? 'bg-amber-500 hover:bg-amber-400 text-white'
                  : 'bg-amber-600 hover:bg-amber-500 text-white'
              }`}
              title="Your profile has changed since this page was personalized"
            >
              <RotateCcw className="w-4 h-4" />
              Re-personalize
            </button>
          )}
        </div>
      </div>
    );
  }

  // Idle state - show personalize option
  // Check if there's a cached version available
  if (hasCachedVersion) {
    console.log('[PersonalizationBar] Rendering cached version available UI');
    return (
      <div
        className={`flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4 p-4 rounded-xl mb-6 ${
          isDark
            ? 'bg-gradient-to-r from-emerald-500/10 to-amber-500/10 border border-emerald-500/20'
            : 'bg-gradient-to-r from-emerald-50 to-amber-50 border border-emerald-200'
        }`}
      >
        <div className="flex items-start gap-3">
          <div
            className={`flex items-center justify-center w-10 h-10 rounded-lg flex-shrink-0 ${
              isDark ? 'bg-emerald-500/20' : 'bg-emerald-100'
            }`}
          >
            <Sparkles
              className={`w-5 h-5 ${isDark ? 'text-emerald-400' : 'text-emerald-600'}`}
            />
          </div>
          <div>
            <p
              className={`font-semibold ${
                isDark ? 'text-slate-200' : 'text-slate-800'
              }`}
            >
              Personalized version available
            </p>
            <p
              className={`text-sm ${isDark ? 'text-slate-400' : 'text-slate-600'}`}
            >
              {isProfileStale
                ? 'Your profile has changed - re-personalize for free'
                : 'You previously personalized this page'}
            </p>
          </div>
        </div>
        <div className="flex gap-2">
          <button
            onClick={handleLoadCached}
            className={`flex items-center gap-2 px-4 py-2 rounded-lg text-sm font-semibold transition-all duration-200 ${
              isDark
                ? 'bg-emerald-500 hover:bg-emerald-400 text-white shadow-lg shadow-emerald-500/20'
                : 'bg-emerald-600 hover:bg-emerald-500 text-white shadow-lg shadow-emerald-600/20'
            }`}
          >
            <Eye className="w-4 h-4" />
            View Personalized
          </button>
          {isProfileStale && (
            <button
              onClick={handlePersonalizeClick}
              className={`flex items-center gap-2 px-4 py-2 rounded-lg text-sm font-semibold transition-all duration-200 ${
                isDark
                  ? 'bg-amber-500 hover:bg-amber-400 text-white'
                  : 'bg-amber-600 hover:bg-amber-500 text-white'
              }`}
            >
              <RotateCcw className="w-4 h-4" />
              Re-personalize (Free)
            </button>
          )}
        </div>
      </div>
    );
  }

  // Default idle state - offer to personalize
  console.log('[PersonalizationBar] Rendering idle state with Personalize button');
  return (
    <div
      className={`flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4 p-4 rounded-xl mb-6 ${
        isDark
          ? 'bg-gradient-to-r from-emerald-500/10 to-amber-500/10 border border-emerald-500/20'
          : 'bg-gradient-to-r from-emerald-50 to-amber-50 border border-emerald-200'
      }`}
    >
      <div className="flex items-start gap-3">
        <div
          className={`flex items-center justify-center w-10 h-10 rounded-lg flex-shrink-0 ${
            isDark ? 'bg-emerald-500/20' : 'bg-emerald-100'
          }`}
        >
          <Sparkles
            className={`w-5 h-5 ${isDark ? 'text-emerald-400' : 'text-emerald-600'}`}
          />
        </div>
        <div>
          <p
            className={`font-semibold ${
              isDark ? 'text-slate-200' : 'text-slate-800'
            }`}
          >
            Personalize this page?
          </p>
          <p
            className={`text-sm ${isDark ? 'text-slate-400' : 'text-slate-600'}`}
          >
            We can tailor content based on your{' '}
            <span className={`font-medium ${isDark ? 'text-emerald-400' : 'text-emerald-600'}`}>
              {profile.tech_background?.replace('_', ' ') || 'profile'}
            </span>{' '}
            background
            {quotaStatus && (
              <span className="ml-1">
                ({quotaStatus.remaining}/{quotaStatus.limit} left today)
              </span>
            )}
          </p>
        </div>
      </div>
      <button
        onClick={(e) => {
          console.log('[PersonalizationBar] Button onClick fired!', e);
          handlePersonalizeClick();
        }}
        disabled={quotaStatus?.remaining === 0}
        className={`flex items-center gap-2 px-4 py-2 rounded-lg text-sm font-semibold transition-all duration-200 ${
          quotaStatus?.remaining === 0
            ? isDark
              ? 'bg-slate-600 text-slate-400 cursor-not-allowed'
              : 'bg-slate-300 text-slate-500 cursor-not-allowed'
            : isDark
              ? 'bg-emerald-500 hover:bg-emerald-400 text-white shadow-lg shadow-emerald-500/20'
              : 'bg-emerald-600 hover:bg-emerald-500 text-white shadow-lg shadow-emerald-600/20'
        } focus:outline-none focus:ring-2 focus:ring-emerald-500 focus:ring-offset-2 ${
          isDark ? 'focus:ring-offset-slate-900' : 'focus:ring-offset-white'
        }`}
      >
        <Wand2 className="w-4 h-4" />
        Personalize Page
      </button>
    </div>
  );
};

export default PersonalizationBar;
