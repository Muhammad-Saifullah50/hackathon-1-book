// website/src/data/profile-schema.ts

import { z } from 'zod';

// Enums matching backend Pydantic models
export const AgeRangeSchema = z.enum(['under_18', '18_24', '25_34', '35_plus']);
export const EducationLevelSchema = z.enum(['high_school', 'undergrad', 'masters', 'phd', 'self_taught']);
export const TechBackgroundSchema = z.enum(['software_engineer', 'hardware_engineer', 'student', 'hobbyist']);
export const PrimaryGoalSchema = z.enum(['career_switch', 'academic', 'hobby_project', 'startup_founder']);
export const LearningModeSchema = z.enum(['visual', 'textual', 'code_first']);
export const LearningSpeedSchema = z.enum(['intensive', 'balanced', 'casual']);

export const UserProfileSchema = z.object({
  user_id: z.string().uuid(), // UUID as string on frontend
  age_range: AgeRangeSchema.optional(),
  education_level: EducationLevelSchema.optional(),
  tech_background: TechBackgroundSchema.optional(),
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
