import { test, expect } from '@playwright/test';

test.describe('Responsive Layout', () => {
  test.use({ viewport: { width: 375, height: 667 } }); // iPhone SE size

  test('should show hamburger menu on mobile', async ({ page }) => {
    await page.goto('/');
    
    // Docusaurus default mobile menu toggle
    const toggle = page.getByLabel('Navigation bar toggle');
    await expect(toggle).toBeVisible();
    
    await toggle.click();
    await expect(page.getByRole('link', { name: /Tutorial/i })).toBeVisible(); 
  });

  test('should collapse sidebar on mobile docs page', async ({ page }) => {
    await page.goto('/docs/intro');
    // Sidebar should be hidden or accessible via a menu, not statically visible on side
    // Docusaurus typically puts sidebar in the mobile nav menu or a "breadcrumbs" style dropdown
    // This assertion checks that the desktop sidebar is NOT visible in the viewport in its normal place
    // Note: Specific selectors depend on Docusaurus theme structure
    // A rough check is that main content takes up most width
    const main = page.locator('main');
    const box = await main.boundingBox();
    expect(box?.width).toBeGreaterThan(300);
  });
});
