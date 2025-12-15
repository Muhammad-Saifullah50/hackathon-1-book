// website/src/types/personalization.ts
/**
 * TypeScript types for page personalization feature.
 */

/**
 * Cached personalized page stored in localStorage.
 */
export interface PersonalizedPageCache {
  /** Personalized markdown content */
  content: string;
  /** SHA-256 hash of user profile at personalization time */
  profileHash: string;
  /** SHA-256 hash of original content (for change detection) */
  originalContentHash: string;
  /** ISO timestamp when personalized */
  timestamp: string;
  /** Page URL path */
  pageUrl: string;
  /** ISO timestamp of last access (for LRU eviction) */
  lastAccessed: string;
  /** Original content (for toggle feature) */
  originalContent: string;
}

/**
 * User's daily personalization quota status.
 */
export interface QuotaStatus {
  /** Maximum personalizations per day */
  limit: number;
  /** Personalizations used today */
  used: number;
  /** Personalizations remaining today */
  remaining: number;
  /** ISO timestamp when quota resets (midnight UTC) */
  resetsAt: string;
}

/**
 * Request payload for personalization API.
 */
export interface PersonalizationRequest {
  /** URL path of the page being personalized */
  pageUrl: string;
  /** Original markdown content of the page */
  pageContent: string;
  /** If true, this is a free re-personalization (doesn't decrement quota) */
  isFreeRepersonalization: boolean;
}

/**
 * Response from personalization API.
 */
export interface PersonalizationResponse {
  /** Personalized markdown content */
  personalizedContent: string;
  /** SHA-256 hash of user profile at personalization time */
  profileHash: string;
  /** SHA-256 hash of original content */
  originalContentHash: string;
  /** Processing time in milliseconds */
  processingTimeMs: number;
  /** Remaining personalizations for today */
  quotaRemaining: number;
}

/**
 * Item in personalization history.
 */
export interface PersonalizationHistoryItem {
  /** Page URL path */
  url: string;
  /** Profile hash at personalization time */
  profileHash: string;
  /** ISO timestamp when first personalized */
  createdAt: string;
}

/**
 * Response from history API.
 */
export interface PersonalizationHistoryResponse {
  /** List of personalized pages */
  pages: PersonalizationHistoryItem[];
  /** Current user profile hash (for staleness detection) */
  currentProfileHash: string;
}

/**
 * Response for single page history check.
 */
export interface PageHistoryResponse {
  /** Whether the page was previously personalized */
  found: boolean;
  /** Profile hash at personalization time (if found) */
  profileHash?: string;
  /** Original content hash (if found) */
  originalContentHash?: string;
  /** ISO timestamp when personalized (if found) */
  createdAt?: string;
  /** True if profile has changed since personalization */
  isStale?: boolean;
}

/**
 * Error response from personalization API.
 */
export interface PersonalizationError {
  /** Error code */
  error: string;
  /** Human-readable error message */
  message: string;
  /** Additional error details */
  details?: Record<string, unknown>;
  /** Profile wizard URL (for profile_incomplete error) */
  profileWizardUrl?: string;
  /** Quota status (for quota_exceeded error) */
  quotaStatus?: QuotaStatus;
}

/**
 * Personalization state for usePersonalization hook.
 */
export type PersonalizationState =
  | 'idle'
  | 'loading'
  | 'personalized'
  | 'error'
  | 'quota_exceeded';

/**
 * View mode for toggling between original and personalized content.
 */
export type ViewMode = 'original' | 'personalized';
