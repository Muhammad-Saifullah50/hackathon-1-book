// website/src/data/profile-schema.ts

import { z } from 'zod';

// Enums matching database CHECK constraints
export const AgeRangeSchema = z.enum(['under_18', '18_24', '25_34', '35_44', '45_plus']);
export const EducationLevelSchema = z.enum(['high_school', 'bachelors', 'masters', 'phd', 'self_taught']);
export const TechBackgroundSchema = z.enum(['none', 'beginner', 'intermediate', 'advanced']);
export const FocusAreaSchema = z.enum(['hardware', 'software']);
export const PrimaryGoalSchema = z.enum(['career', 'research', 'hobby', 'education']);
export const LearningModeSchema = z.enum(['visual', 'reading', 'hands_on', 'mixed']);
export const LearningSpeedSchema = z.enum(['thorough', 'balanced', 'accelerated']);

export const UserProfileSchema = z.object({
  user_id: z.string(), // Better Auth uses NanoID strings, not UUIDs
  age_range: AgeRangeSchema.optional(),
  education_level: EducationLevelSchema.optional(),
  tech_background: TechBackgroundSchema.optional(),
  focus_area: FocusAreaSchema.optional(),
  primary_goal: PrimaryGoalSchema.optional(),
  learning_mode: LearningModeSchema.optional(),
  learning_speed: LearningSpeedSchema.optional(),
  time_per_week: z.number().int().nonnegative().optional(),
  preferred_language: z.string().default('en'),
});

// For partial updates, all fields become optional
export const UserProfileUpdateSchema = UserProfileSchema.partial();

export type UserProfile = z.infer<typeof UserProfileSchema>;
export type UserProfileUpdate = z.infer<typeof UserProfileUpdateSchema>;
