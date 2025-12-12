// tests/e2e/personalization_bar.spec.ts
import { test, expect, Page, Dialog } from '@playwright/test';

const BASE_URL = 'http://localhost:3000';
const BACKEND_URL = 'http://localhost:8000';
const DOC_PAGE = '/docs/module-01/overview';

test.describe('Personalization Bar', () => {
  // Helper to create a user and profile via API
  async function setupAuthenticatedUser(page: Page) {
    const timestamp = Date.now();
    const email = `e2e-test-${timestamp}@example.com`;
    const password = 'TestPassword123!';

    // Sign up via API
    const signupResponse = await page.request.post(`${BACKEND_URL}/auth/signup`, {
      data: { email, password }
    });

    // If signup succeeded and we got a token, use it
    if (signupResponse.ok()) {
      const signupData = await signupResponse.json();
      if (signupData.access_token) {
        // Store token in localStorage
        await page.goto(BASE_URL);
        await page.evaluate((token: string) => {
          localStorage.setItem('access_token', token);
        }, signupData.access_token);

        // Create a profile
        await page.request.post(`${BACKEND_URL}/profile`, {
          headers: { Authorization: `Bearer ${signupData.access_token}` },
          data: {
            user_id: signupData.user.id,
            tech_background: 'software_engineer',
            time_per_week: 10
          }
        });

        return { email, password, token: signupData.access_token, userId: signupData.user.id };
      }
    }

    return null;
  }

  test('should NOT display the personalization bar on non-doc pages', async ({ page }) => {
    // Navigate to home page (non-doc page)
    await page.goto(BASE_URL);

    // PersonalizationBar should NOT be visible on non-doc pages
    const personalizeButton = page.getByRole('button', { name: /Personalize Page/i });
    await expect(personalizeButton).not.toBeVisible();
  });

  test('should NOT display the personalization bar for unauthenticated users', async ({ page }) => {
    // Clear any stored auth
    await page.goto(BASE_URL);
    await page.evaluate(() => {
      localStorage.removeItem('access_token');
    });

    // Navigate to a doc page
    await page.goto(`${BASE_URL}${DOC_PAGE}`);

    // Wait for page to load
    await page.waitForLoadState('networkidle');

    // PersonalizationBar should NOT be visible without authentication/profile
    const personalizeButton = page.getByRole('button', { name: /Personalize Page/i });
    await expect(personalizeButton).not.toBeVisible();
  });

  test('should display the personalization bar for authenticated user with profile on doc pages', async ({ page }) => {
    const user = await setupAuthenticatedUser(page);

    if (!user) {
      test.skip(true, 'Could not set up authenticated user - backend may not be running');
      return;
    }

    // Navigate to a doc page
    await page.goto(`${BASE_URL}${DOC_PAGE}`);
    await page.waitForLoadState('networkidle');

    // Wait for profile to load and bar to appear
    const personalizeButton = page.getByRole('button', { name: /Personalize Page/i });
    await expect(personalizeButton).toBeVisible({ timeout: 10000 });
    await expect(page.getByText(/Personalize this page/i)).toBeVisible();
  });

  test('should trigger personalization alert when clicking the button', async ({ page }) => {
    const user = await setupAuthenticatedUser(page);

    if (!user) {
      test.skip(true, 'Could not set up authenticated user - backend may not be running');
      return;
    }

    // Navigate to a doc page
    await page.goto(`${BASE_URL}${DOC_PAGE}`);
    await page.waitForLoadState('networkidle');

    const personalizeButton = page.getByRole('button', { name: /Personalize Page/i });
    await expect(personalizeButton).toBeVisible({ timeout: 10000 });

    // Set up dialog handler before clicking
    let dialogMessage = '';
    page.on('dialog', async (dialog: Dialog) => {
      dialogMessage = dialog.message();
      await dialog.dismiss();
    });

    await personalizeButton.click();

    // Wait for dialog to be handled
    await page.waitForTimeout(500);

    // Verify the alert was triggered
    expect(dialogMessage).toContain('Personalization logic triggered!');
  });

  test('should show user tech background in personalization bar', async ({ page }) => {
    const user = await setupAuthenticatedUser(page);

    if (!user) {
      test.skip(true, 'Could not set up authenticated user - backend may not be running');
      return;
    }

    // Navigate to a doc page
    await page.goto(`${BASE_URL}${DOC_PAGE}`);
    await page.waitForLoadState('networkidle');

    // Check that the tech background is displayed (we set 'software_engineer' in setup)
    await expect(page.getByText(/software engineer/i)).toBeVisible({ timeout: 10000 });
  });
});
