---
id: 0001
title: Create Authentication & Detailed Profiling Spec
stage: spec
date: 2025-12-09
surface: agent
model: gemini
feature: 008-learner-auth-profile
branch: 008-learner-auth-profile
user: (unknown)
command: /sp.specify # Feature Spec: Authentication & Detailed Profiling ## 1. Role & Goal * **Goal:** Create a deep "Learner DNA" profile for every user. * **Why:** To enable granular customization of the book's content, pacing, and language via the "Personalize Page" bar. ## 2. Technical Stack * **Auth Engine:** **`better-auth`** (with custom schema extensions). * **Database:** **Supabase** (PostgreSQL). * **UI Components:** **Shadcn UI** (Multi-step Wizard Form for signup). ## 3. Mandatory Documentation Protocol (Context 7) * **Source of Truth:** Retrieve `better-auth` plugin documentation from `context7`. * **Schema Validation:** Use **Zod** to strictly validate all enum fields (e.g., ensuring "Learning Mode" is only "Visual" or "Text", not random strings). ## 4. The Signup Experience (User Stories) * **Story: The "Learner Profile" Wizard** * As a new user, I want a multi-step signup process that feels like setting up a game character, not filing taxes. * **Step 1: The Basics** (Email, Password, Age Range). * **Step 2: The Background** (Education Level, Tech Stack History). * **Step 3: The Strategy** (Goals, Time per Week, Learning Speed). * **Story: The "Personalize" Trigger** * As a user reading Chapter 1, I want to see a persistent **Personalization Bar** at the top. * When I click "Personalize Page," the system should use my *Time Per Week* and *Tech Background* to filter out non-essential paragraphs (logic to be implemented later). ## 5. Expanded Database Schema (Supabase) You must extend the `user` table (or create a linked `profiles` table) with these specific columns: | Field | Type | Options / Description | | :--- | :--- | :--- | | `age_range` | Enum | `'under_18'`, `'18_24'`, `'25_34'`, `'35_plus'` | | `education_level` | Enum | `'high_school'`, `'undergrad'`, `'masters'`, `'phd'`, `'self_taught'` | | `tech_background` | Enum | `'software_engineer'`, `'hardware_engineer'`, `'student'`, `'hobbyist'` | | `primary_goal` | Enum | `'career_switch'`, `'academic'`, `'hobby_project'`, `'startup_founder'` | | `learning_mode` | Enum | `'visual'` (more diagrams), `'textual'` (more reading), `'code_first'` (show me the code) | | `learning_speed` | Enum | `'intensive'`, `'balanced'`, `'casual'` (high-level) | | `time_per_week` | Integer | Hours available (e.g., 5, 10, 20) | | `preferred_language`| String | Default `'en'`, supports `'ur'` (Urdu), etc. | ## 6. Development Rules * **Progressive Profiling:** If the signup form is too long, implement a "Skip for now" option, but highlight that personalization features will be disabled until completed. * **Privacy:** Explicitly state that this data is used *only* for tailoring the textbook experience.
labels: [spec, auth, profiling]
links:
  spec: specs/008-learner-auth-profile/spec.md
  ticket: null
  adr: null
  pr: null
files:
- specs/008-learner-auth-profile/spec.md
- specs/008-learner-auth-profile/checklists/requirements.md
tests:

---

## Prompt

# Feature Spec: Authentication & Detailed Profiling ## 1. Role & Goal * **Goal:** Create a deep "Learner DNA" profile for every user. * **Why:** To enable granular customization of the book's content, pacing, and language via the "Personalize Page" bar. ## 2. Technical Stack * **Auth Engine:** **`better-auth`** (with custom schema extensions). * **Database:** **Supabase** (PostgreSQL). * **UI Components:** **Shadcn UI** (Multi-step Wizard Form for signup). ## 3. Mandatory Documentation Protocol (Context 7) * **Source of Truth:** Retrieve `better-auth` plugin documentation from `context7`. * **Schema Validation:** Use **Zod** to strictly validate all enum fields (e.g., ensuring "Learning Mode" is only "Visual" or "Text", not random strings). ## 4. The Signup Experience (User Stories) * **Story: The "Learner Profile" Wizard** * As a new user, I want a multi-step signup process that feels like setting up a game character, not filing taxes. * **Step 1: The Basics** (Email, Password, Age Range). * **Step 2: The Background** (Education Level, Tech Stack History). * **Step 3: The Strategy** (Goals, Time per Week, Learning Speed). * **Story: The "Personalize" Trigger** * As a user reading Chapter 1, I want to see a persistent **Personalization Bar** at the top. * When I click "Personalize Page," the system should use my *Time Per Week* and *Tech Background* to filter out non-essential paragraphs (logic to be implemented later). ## 5. Expanded Database Schema (Supabase) You must extend the `user` table (or create a linked `profiles` table) with these specific columns: | Field | Type | Options / Description | | :--- | :--- | :--- | | `age_range` | Enum | `'under_18'`, `'18_24'`, `'25_34'`, `'35_plus'` | | `education_level` | Enum | `'high_school'`, `'undergrad'`, `'masters'`, `'phd'`, `'self_taught'` | | `tech_background` | Enum | `'software_engineer'`, `'hardware_engineer'`, `'student'`, `'hobbyist'` | | `primary_goal` | Enum | `'career_switch'`, `'academic'`, `'hobby_project'`, `'startup_founder'` | | `learning_mode` | Enum | `'visual'` (more diagrams), `'textual'` (more reading), `'code_first'` (show me the code) | | `learning_speed` | Enum | `'intensive'`, `'balanced'`, `'casual'` (high-level) | | `time_per_week'` | Integer | Hours available (e.g., 5, 10, 20) | | `preferred_language`| String | Default `'en'`, supports `'ur'` (Urdu), etc. | ## 6. Development Rules * **Progressive Profiling:** If the signup form is too long, implement a "Skip for now" option, but highlight that personalization features will be disabled until completed. * **Privacy:** Explicitly state that this data is used *only* for tailoring the textbook experience.

## Response snapshot

The feature specification for "Authentication & Detailed Profiling" has been successfully created and validated.

**Branch**: `008-learner-auth-profile`
**Spec File**: `specs/008-learner-auth-profile/spec.md`
**Checklist**: `specs/008-learner-auth-profile/checklists/requirements.md` (All items passed)

The specification is now ready for the next phase, which could be `/sp.clarify` for any remaining questions (though none were identified by the checklist) or `/sp.plan` to begin architectural planning.

## Outcome

- ✅ Impact: The feature specification for "Authentication & Detailed Profiling" has been created and validated.
- 🧪 Tests: No tests were run or added in this stage.
- 📁 Files: specs/008-learner-auth-profile/spec.md, specs/008-learner-auth-profile/checklists/requirements.md have been created/updated.
- 🔁 Next prompts: User can now proceed with /sp.clarify or /sp.plan.
- 🧠 Reflection: The spec was created according to user's prompt, including specific technical constraints. The checklist helped to ensure completeness.

## Evaluation notes (flywheel)

- Failure modes observed: `create-new-feature.sh` reported existing branch, necessitating manual checkout.
- Graders run and results (PASS/FAIL): (not applicable)
- Prompt variant (if applicable): (not applicable)
- Next experiment (smallest change to try): Ensure `create-new-feature.sh` handles existing branches/directories gracefully when invoked with `--number` and `--short-name`.
