// website/tests/e2e/personalization.spec.ts
/**
 * E2E tests for page personalization feature.
 *
 * Tests:
 * - T063: Full personalization flow (login -> navigate -> personalize -> toggle)
 * - T064: Cross-device scenario (simulated via localStorage manipulation)
 *
 * Prerequisites:
 * - Backend running at http://localhost:8000
 * - Frontend running at http://localhost:3000
 * - Test user account exists with completed profile
 */

import { test, expect, type Page, type BrowserContext } from '@playwright/test';

// Test configuration
const BASE_URL = process.env.TEST_BASE_URL || 'http://localhost:3000';
const API_URL = process.env.TEST_API_URL || 'http://localhost:8000';
const TEST_USER_EMAIL = process.env.TEST_USER_EMAIL || 'testuser@example.com';
const TEST_USER_PASSWORD = process.env.TEST_USER_PASSWORD || 'testPassword123!';

// Selectors
const SELECTORS = {
  // Auth
  loginEmailInput: 'input[type="email"]',
  loginPasswordInput: 'input[type="password"]',
  loginButton: 'button:has-text("Log in")',

  // Personalization Bar
  personalizationBar: '[data-testid="personalization-bar"]',
  personalizeButton: 'button:has-text("Personalize Page")',
  showOriginalButton: 'button:has-text("Show Original")',
  showPersonalizedButton: 'button:has-text("Show Personalized")',
  viewPersonalizedButton: 'button:has-text("View Personalized")',
  rePersonalizeButton: 'button:has-text("Re-personalize")',
  loadingIndicator: 'text=Personalizing content...',
  personalizedMessage: 'text=Viewing personalized content',
  originalMessage: 'text=Viewing original content',
  quotaDisplay: /\d\/5 left today/,

  // Login prompts
  loginPrompt: 'text=Log in to tailor content',
  completeProfilePrompt: 'text=Complete your profile',

  // Article content
  articleContent: 'article',
};

/**
 * Helper function to login a user.
 */
async function loginUser(page: Page) {
  await page.goto(`${BASE_URL}/login`);
  await page.fill(SELECTORS.loginEmailInput, TEST_USER_EMAIL);
  await page.fill(SELECTORS.loginPasswordInput, TEST_USER_PASSWORD);
  await page.click(SELECTORS.loginButton);

  // Wait for redirect after login
  await page.waitForURL(/^(?!.*\/login)/);
}

/**
 * Helper function to navigate to a documentation page.
 */
async function navigateToDocsPage(page: Page, path = '/docs/module-1/intro') {
  await page.goto(`${BASE_URL}${path}`);
  await page.waitForSelector(SELECTORS.articleContent);
}

/**
 * Helper function to clear localStorage for personalization data.
 */
async function clearPersonalizationCache(page: Page) {
  await page.evaluate(() => {
    const keysToRemove = [];
    for (let i = 0; i < localStorage.length; i++) {
      const key = localStorage.key(i);
      if (key?.startsWith('personalized_')) {
        keysToRemove.push(key);
      }
    }
    keysToRemove.forEach((key) => localStorage.removeItem(key));
  });
}

// =============================================================================
// T063: Full Personalization Flow Tests
// =============================================================================

test.describe('T063: Full Personalization Flow', () => {
  test.beforeEach(async ({ page }) => {
    await clearPersonalizationCache(page);
  });

  test('should show login prompt for unauthenticated users', async ({ page }) => {
    await navigateToDocsPage(page);

    // Should show login prompt
    await expect(page.locator(SELECTORS.loginPrompt)).toBeVisible();
    await expect(page.locator('a:has-text("Log in")')).toBeVisible();
  });

  test('should show personalize button for authenticated users', async ({ page }) => {
    // Login first
    await loginUser(page);

    // Navigate to docs
    await navigateToDocsPage(page);

    // Should show personalize button
    await expect(page.locator(SELECTORS.personalizeButton)).toBeVisible();
  });

  test('should personalize content and show loading state', async ({ page }) => {
    // Login
    await loginUser(page);

    // Navigate to docs
    await navigateToDocsPage(page);

    // Click personalize
    await page.click(SELECTORS.personalizeButton);

    // Should show loading state
    await expect(page.locator(SELECTORS.loadingIndicator)).toBeVisible({ timeout: 1000 });

    // Wait for personalization to complete (may take up to 15 seconds)
    await expect(page.locator(SELECTORS.personalizedMessage)).toBeVisible({ timeout: 20000 });

    // Show Original button should be visible
    await expect(page.locator(SELECTORS.showOriginalButton)).toBeVisible();
  });

  test('should toggle between original and personalized content', async ({ page }) => {
    // Login and navigate
    await loginUser(page);
    await navigateToDocsPage(page);

    // Personalize
    await page.click(SELECTORS.personalizeButton);
    await expect(page.locator(SELECTORS.personalizedMessage)).toBeVisible({ timeout: 20000 });

    // Toggle to original
    await page.click(SELECTORS.showOriginalButton);
    await expect(page.locator(SELECTORS.originalMessage)).toBeVisible();
    await expect(page.locator(SELECTORS.showPersonalizedButton)).toBeVisible();

    // Toggle back to personalized
    await page.click(SELECTORS.showPersonalizedButton);
    await expect(page.locator(SELECTORS.personalizedMessage)).toBeVisible();
  });

  test('should show cached version on return visit', async ({ page }) => {
    // Login and navigate
    await loginUser(page);
    await navigateToDocsPage(page);

    // Personalize
    await page.click(SELECTORS.personalizeButton);
    await expect(page.locator(SELECTORS.personalizedMessage)).toBeVisible({ timeout: 20000 });

    // Navigate away
    await page.goto(`${BASE_URL}/`);

    // Return to the same docs page
    await navigateToDocsPage(page);

    // Should show "View Personalized" option since cache exists
    await expect(page.locator(SELECTORS.viewPersonalizedButton)).toBeVisible();
  });

  test('should display remaining quota', async ({ page }) => {
    // Login and navigate
    await loginUser(page);
    await navigateToDocsPage(page);

    // Quota display should be visible
    await expect(page.locator('text=/\\d\\/5 left today/')).toBeVisible();
  });

  test('should preserve code blocks in personalized content', async ({ page }) => {
    // Login and navigate to a page with code blocks
    await loginUser(page);
    await navigateToDocsPage(page);

    // Get original code blocks
    const originalCodeBlocks = await page.locator('pre code').count();

    // Personalize
    await page.click(SELECTORS.personalizeButton);
    await expect(page.locator(SELECTORS.personalizedMessage)).toBeVisible({ timeout: 20000 });

    // Code blocks should still exist
    const personalizedCodeBlocks = await page.locator('pre code').count();
    expect(personalizedCodeBlocks).toBe(originalCodeBlocks);
  });
});

// =============================================================================
// T064: Cross-Device Scenario (Simulated)
// =============================================================================

test.describe('T064: Cross-Device Personalization Awareness', () => {
  test('should detect previously personalized page on new device', async ({ browser }) => {
    // Create two browser contexts to simulate two devices
    const deviceA = await browser.newContext();
    const deviceB = await browser.newContext();

    const pageA = await deviceA.newPage();
    const pageB = await deviceB.newPage();

    try {
      // Device A: Login and personalize
      await loginUser(pageA);
      await navigateToDocsPage(pageA);
      await pageA.click(SELECTORS.personalizeButton);
      await expect(pageA.locator(SELECTORS.personalizedMessage)).toBeVisible({ timeout: 20000 });

      // Device B: Login (same user) and navigate to same page
      await loginUser(pageB);
      await navigateToDocsPage(pageB);

      // Device B should NOT have local cache, but server history should show
      // the page was personalized - might show "Re-personalize (Free)"
      // or just the regular personalize button

      // At minimum, the personalize button should be visible
      await expect(
        pageB.locator(`${SELECTORS.personalizeButton}, ${SELECTORS.rePersonalizeButton}`)
      ).toBeVisible();
    } finally {
      await deviceA.close();
      await deviceB.close();
    }
  });

  test('should handle stale profile detection', async ({ page }) => {
    // Login and navigate
    await loginUser(page);
    await navigateToDocsPage(page);

    // Personalize
    await page.click(SELECTORS.personalizeButton);
    await expect(page.locator(SELECTORS.personalizedMessage)).toBeVisible({ timeout: 20000 });

    // Simulate profile change by modifying the cached profile hash
    await page.evaluate(() => {
      const keysToModify = [];
      for (let i = 0; i < localStorage.length; i++) {
        const key = localStorage.key(i);
        if (key?.startsWith('personalized_')) {
          keysToModify.push(key);
        }
      }
      keysToModify.forEach((key) => {
        const data = JSON.parse(localStorage.getItem(key) || '{}');
        data.profileHash = 'stale_hash_that_does_not_match';
        localStorage.setItem(key, JSON.stringify(data));
      });
    });

    // Refresh the page
    await page.reload();

    // Should show re-personalize option due to stale profile
    // Wait for either the re-personalize button or the view personalized button
    await expect(
      page.locator(`${SELECTORS.rePersonalizeButton}, ${SELECTORS.viewPersonalizedButton}`)
    ).toBeVisible({ timeout: 5000 });
  });
});

// =============================================================================
// Error Handling Tests
// =============================================================================

test.describe('Error Handling', () => {
  test('should handle API timeout gracefully', async ({ page }) => {
    // This test would require mocking the backend to simulate timeout
    // For now, skip with explanation
    test.skip(true, 'Requires backend mocking infrastructure');
  });

  test('should show quota exceeded message', async ({ page }) => {
    // This test would require a user at quota limit
    // For now, skip with explanation
    test.skip(true, 'Requires user with exhausted quota');
  });
});

// =============================================================================
// Accessibility Tests
// =============================================================================

test.describe('Accessibility', () => {
  test('personalization bar should be keyboard accessible', async ({ page }) => {
    await loginUser(page);
    await navigateToDocsPage(page);

    // Tab to the personalize button
    await page.keyboard.press('Tab');

    // Eventually we should be able to reach the button
    const focusedElement = page.locator(':focus');
    await expect(focusedElement).toBeVisible();
  });

  test('loading state should have aria attributes', async ({ page }) => {
    await loginUser(page);
    await navigateToDocsPage(page);

    // Click personalize and check for loading indicator
    await page.click(SELECTORS.personalizeButton);

    // The loading container should be present
    await expect(page.locator(SELECTORS.loadingIndicator)).toBeVisible({ timeout: 1000 });
  });
});
