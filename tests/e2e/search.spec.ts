import { test, expect } from '@playwright/test';

test.describe('Global Search', () => {
  test('should open search modal on click', async ({ page }) => {
    await page.goto('/');
    // Docusaurus default search button usually has this class or aria-label
    // Adjust selector based on actual implementation (Algolia or Local Search)
    const searchButton = page.getByLabel('Search');
    await expect(searchButton).toBeVisible();
    
    await searchButton.click();
    const modal = page.locator('.DocSearch-Modal');
    await expect(modal).toBeVisible();
  });

  test('should return results for "URDF"', async ({ page }) => {
    await page.goto('/');
    await page.getByLabel('Search').click();
    await page.getByPlaceholder('Search docs').fill('URDF');
    
    // Wait for results
    const results = page.locator('.DocSearch-Hit');
    await expect(results.first()).toBeVisible();
  });
});
