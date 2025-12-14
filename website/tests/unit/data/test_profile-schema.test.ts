// website/tests/unit/data/test_profile-schema.test.ts
import { UserProfileSchema, UserProfileUpdateSchema, FocusAreaSchema } from '../../../src/data/profile-schema';
import { z } from 'zod';
import { v4 as uuidv4 } from 'uuid';

describe('UserProfileSchema', () => {
  it('should validate a complete user profile', () => {
    const validProfile = {
      user_id: uuidv4(),
      age_range: '18_24',
      education_level: 'undergrad',
      tech_background: 'software_engineer',
      primary_goal: 'career_switch',
      learning_mode: 'visual',
      learning_speed: 'intensive',
      time_per_week: 15,
      preferred_language: 'en',
    };
    expect(() => UserProfileSchema.parse(validProfile)).not.toThrow();
  });

  it('should validate a minimal user profile with default language', () => {
    const minimalProfile = {
      user_id: uuidv4(),
    };
    const parsed = UserProfileSchema.parse(minimalProfile);
    expect(parsed.preferred_language).toBe('en');
    expect(parsed.age_range).toBeUndefined();
  });

  it('should fail validation for invalid UUID', () => {
    const invalidProfile = {
      user_id: 'not-a-uuid',
    };
    expect(() => UserProfileSchema.parse(invalidProfile)).toThrow(z.ZodError);
  });

  it('should fail validation for invalid enum value', () => {
    const invalidProfile = {
      user_id: uuidv4(),
      age_range: 'invalid-age', // Invalid enum value
    };
    expect(() => UserProfileSchema.parse(invalidProfile)).toThrow(z.ZodError);
  });

  it('should fail validation for negative time_per_week', () => {
    const invalidProfile = {
      user_id: uuidv4(),
      time_per_week: -10,
    };
    expect(() => UserProfileSchema.parse(invalidProfile)).toThrow(z.ZodError);
  });

  it('should validate UserProfileUpdateSchema for partial updates', () => {
    const partialUpdate = {
      time_per_week: 20,
      learning_speed: 'casual',
    };
    expect(() => UserProfileUpdateSchema.parse(partialUpdate)).not.toThrow();
    
    const anotherPartialUpdate = {
      age_range: '35_plus',
      preferred_language: 'fr',
    };
    expect(() => UserProfileUpdateSchema.parse(anotherPartialUpdate)).not.toThrow();
  });

  it('should fail UserProfileUpdateSchema for invalid enum value', () => {
    const invalidUpdate = {
      education_level: 'masterss', // Typo, invalid enum
    };
    expect(() => UserProfileUpdateSchema.parse(invalidUpdate)).toThrow(z.ZodError);
  });
});

describe('FocusAreaSchema', () => {
  it('should validate hardware as a valid focus area', () => {
    expect(() => FocusAreaSchema.parse('hardware')).not.toThrow();
  });

  it('should validate software as a valid focus area', () => {
    expect(() => FocusAreaSchema.parse('software')).not.toThrow();
  });

  it('should reject invalid focus area values', () => {
    expect(() => FocusAreaSchema.parse('both')).toThrow(z.ZodError);
    expect(() => FocusAreaSchema.parse('fullstack')).toThrow(z.ZodError);
    expect(() => FocusAreaSchema.parse('')).toThrow(z.ZodError);
  });

  it('should include focus_area in UserProfileSchema', () => {
    const profileWithFocusArea = {
      user_id: uuidv4(),
      focus_area: 'hardware',
    };
    const parsed = UserProfileSchema.parse(profileWithFocusArea);
    expect(parsed.focus_area).toBe('hardware');
  });

  it('should validate focus_area in UserProfileUpdateSchema', () => {
    const partialUpdate = {
      focus_area: 'software',
    };
    expect(() => UserProfileUpdateSchema.parse(partialUpdate)).not.toThrow();
  });
});