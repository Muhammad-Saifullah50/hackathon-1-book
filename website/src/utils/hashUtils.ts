// website/src/utils/hashUtils.ts
/**
 * Hash utilities for profile and content change detection.
 */

import type { UserProfile } from '../data/profile-schema';

/**
 * Compute SHA-256 hash of user profile attributes.
 * Only includes personalization-relevant fields.
 *
 * @param profile User profile object
 * @returns 64-character hex string (SHA-256 hash)
 */
export async function computeProfileHash(profile: UserProfile): Promise<string> {
  const relevantFields = {
    tech_background: profile.tech_background,
    learning_mode: profile.learning_mode,
    learning_speed: profile.learning_speed,
    preferred_language: profile.preferred_language ?? 'en',
    education_level: profile.education_level,
    primary_goal: profile.primary_goal,
    focus_area: profile.focus_area,
  };

  // Sort keys for deterministic serialization (match Python behavior)
  const sortedKeys = Object.keys(relevantFields).sort();
  const sortedObj: Record<string, unknown> = {};
  for (const key of sortedKeys) {
    sortedObj[key] = relevantFields[key as keyof typeof relevantFields];
  }

  const serialized = JSON.stringify(sortedObj);
  const encoder = new TextEncoder();
  const data = encoder.encode(serialized);
  const hashBuffer = await crypto.subtle.digest('SHA-256', data);
  const hashArray = Array.from(new Uint8Array(hashBuffer));
  return hashArray.map((b) => b.toString(16).padStart(2, '0')).join('');
}

/**
 * Compute SHA-256 hash of content string.
 *
 * @param content Content string to hash
 * @returns 64-character hex string (SHA-256 hash)
 */
export async function computeContentHash(content: string): Promise<string> {
  const encoder = new TextEncoder();
  const data = encoder.encode(content);
  const hashBuffer = await crypto.subtle.digest('SHA-256', data);
  const hashArray = Array.from(new Uint8Array(hashBuffer));
  return hashArray.map((b) => b.toString(16).padStart(2, '0')).join('');
}
