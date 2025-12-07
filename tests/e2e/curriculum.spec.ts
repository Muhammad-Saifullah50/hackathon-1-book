import { test, expect } from '@playwright/test';

test.describe('Curriculum Navigation', () => {
  test('should display the 4 core modules in the sidebar', async ({ page }) => {
    await page.goto('/');
    
    // Check for the main sidebar items corresponding to the 4 modules
    // Note: Actual text depends on docusaurus sidebar configuration, but we expect these keywords
    await expect(page.getByRole('link', { name: /Foundations/i })).toBeVisible();
    await expect(page.getByRole('link', { name: /Digital Twin/i })).toBeVisible();
    await expect(page.getByRole('link', { name: /AI-Robot Brain/i })).toBeVisible();
    await expect(page.getByRole('link', { name: /Embodied Intelligence/i })).toBeVisible();
  });

  test('should navigate to Module 1', async ({ page }) => {
    await page.goto('/');
    await page.getByRole('link', { name: /Foundations/i }).click();
    await expect(page).toHaveURL(/.*module-1/);
    await expect(page.getByRole('heading', { level: 1 })).toBeVisible();
  });
});
