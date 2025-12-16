// website/src/types/translation.ts
/**
 * TypeScript types for Urdu translation feature.
 */

/**
 * Request payload for translation API.
 */
export interface TranslationRequest {
  /** Markdown content to translate */
  page_content: string;
  /** URL path of the page being translated */
  page_url: string;
  /** Source language code (default: "en") */
  source_language?: string;
  /** Target language code (default: "ur") */
  target_language?: string;
}

/**
 * Response from translation API.
 */
export interface TranslationResponse {
  /** Translated markdown content in Urdu */
  translated_content: string;
  /** Source language code */
  source_language: string;
  /** Target language code */
  target_language: string;
  /** Processing time in milliseconds */
  processing_time_ms: number;
  /** SHA-256 hash of original content */
  original_content_hash: string;
  /** Remaining translations for today */
  quota_remaining: number;
  /** Daily translation limit */
  quota_limit: number;
}

/**
 * User's daily translation quota status.
 */
export interface QuotaStatus {
  /** Translations used today */
  used: number;
  /** Translations remaining today */
  remaining: number;
  /** Daily limit */
  limit: number;
  /** ISO timestamp when quota resets (midnight UTC) */
  resets_at: string;
}

/**
 * Response for translation history check.
 */
export interface TranslationHistoryCheck {
  /** Whether page was previously translated */
  has_translation: boolean;
  /** Hash of original content at translation time */
  original_content_hash?: string;
  /** ISO timestamp when translated */
  translated_at?: string;
  /** Whether original content has changed since translation */
  content_changed: boolean;
}

/**
 * Error response from translation API.
 */
export interface TranslationError {
  /** Human-readable error message */
  error: string;
  /** Machine-readable error code */
  code: string;
  /** Additional error context */
  details?: {
    used?: number;
    limit?: number;
    resets_at?: string;
  };
}

/**
 * Source content type for translation.
 * Tracks whether we translated the original page or a personalized version.
 */
export type TranslationSourceType = 'original' | 'personalized';

/**
 * Cached translated page stored in localStorage.
 */
export interface TranslatedPageCache {
  /** Translated markdown content */
  content: string;
  /** SHA-256 hash of original content */
  originalContentHash: string;
  /** ISO timestamp when translated */
  timestamp: string;
  /** Page URL path */
  pageUrl: string;
  /** Target language code (always "ur" for now) */
  targetLanguage: string;
  /** Source content type: original or personalized */
  sourceType: TranslationSourceType;
}

/**
 * Translation state for useTranslation hook.
 */
export type TranslationState =
  | 'idle'           // Initial state, no translation
  | 'loading'        // Translation in progress
  | 'translated'     // Viewing translated content
  | 'error'          // Translation failed
  | 'quota_exceeded'; // Daily limit reached

/**
 * View mode for toggling between original and translated content.
 */
export type ViewMode = 'original' | 'translated';

/**
 * Return type for useTranslation hook.
 */
export interface UseTranslationReturn {
  /** Current translation state */
  state: TranslationState;
  /** Current view mode */
  viewMode: ViewMode;
  /** Error message if any */
  error: string | null;
  /** Current quota status */
  quotaStatus: QuotaStatus | null;
  /** Translated content */
  translatedContent: string | null;
  /** Whether a cached translation exists */
  hasCachedVersion: boolean;
  /** Whether original content has changed since translation */
  isContentStale: boolean;
  /** Source type of the translation (original or personalized) */
  translationSourceType: TranslationSourceType | null;
  /** Translate the current page content */
  translate: (content: string, pageUrl: string, sourceType: TranslationSourceType) => Promise<void>;
  /** Toggle between original and translated views */
  toggleView: () => void;
  /** Load cached translation if available */
  loadCachedVersion: () => void;
  /** Reset translation state (used when personalization view changes) */
  resetTranslation: () => void;
}
