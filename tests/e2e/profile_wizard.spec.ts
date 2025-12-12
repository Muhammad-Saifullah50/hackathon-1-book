// tests/e2e/profile_wizard.spec.ts
import { test, expect } from '@playwright/test';

test.describe('Learner Profile Wizard', () => {
  // Assuming we have a way to seed a user or login before tests
  // For this example, we'll assume the app starts at the login page or redirects to it
  
  test('should complete the full profile wizard flow', async ({ page }) => {
    // 1. Navigate to the wizard page (might need login first in real scenario)
    // Here we assume we can access it directly or are redirected
    await page.goto('http://localhost:3000/signup-wizard');

    // Handle login if redirected (simplified for this example)
    // await page.fill('input[name="email"]', 'test@example.com');
    // await page.fill('input[name="password"]', 'password');
    // await page.click('button:has-text("Login")');

    // 2. Step 1: The Basics
    await expect(page.locator('h2')).toHaveText('Step 1: The Basics');
    await page.selectOption('select#age_range', '18_24');
    await page.click('button:has-text("Next")');

    // 3. Step 2: The Background
    await expect(page.locator('h2')).toHaveText('Step 2: The Background');
    await page.selectOption('select#education_level', 'undergrad');
    await page.selectOption('select#tech_background', 'student');
    await page.click('button:has-text("Next")');

    // 4. Step 3: The Strategy
    await expect(page.locator('h2')).toHaveText('Step 3: The Strategy');
    await page.selectOption('select#primary_goal', 'academic');
    await page.selectOption('select#learning_mode', 'textual');
    await page.selectOption('select#learning_speed', 'balanced');
    await page.fill('input#time_per_week', '15');
    
    // 5. Submit
    await page.click('button:has-text("Finish & Create Profile")');

    // 6. Verify redirection (assuming redirect to home '/')
    await expect(page).toHaveURL('http://localhost:3000/');
  });

  test('should allow skipping the wizard', async ({ page }) => {
    await page.goto('http://localhost:3000/signup-wizard');
    
    await page.click('button:has-text("Skip for now")');
    
    await expect(page).toHaveURL('http://localhost:3000/');
  });

  test('should navigate back and forth between steps', async ({ page }) => {
    await page.goto('http://localhost:3000/signup-wizard');

    // Step 1 -> Step 2
    await page.selectOption('select#age_range', '25_34');
    await page.click('button:has-text("Next")');
    await expect(page.locator('h2')).toHaveText('Step 2: The Background');

    // Step 2 -> Step 1
    await page.click('button:has-text("Back")');
    await expect(page.locator('h2')).toHaveText('Step 1: The Basics');
    
    // Verify data persistence (state check)
    await expect(page.locator('select#age_range')).toHaveValue('25_34');
  });
});
