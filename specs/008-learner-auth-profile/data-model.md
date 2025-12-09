# Data Model: Learner Profile

## Entity: User Profile

This entity represents the extended profile of a learner, capturing various attributes essential for content personalization. It will be linked to the core user authentication system.

### Attributes

| Field               | Type      | Description                                                              | Validation (Zod)                                     |
|---------------------|-----------|--------------------------------------------------------------------------|------------------------------------------------------|
| `user_id`           | UUID      | Foreign key linking to the `auth.users` table (Supabase)                 | Required, UUID format                                |
| `age_range`         | Enum      | Categorization of the user's age                                         | Required, one of `'under_18'`, `'18_24'`, `'25_34'`, `'35_plus'` |
| `education_level`   | Enum      | Highest level of education attained                                      | Required, one of `'high_school'`, `'undergrad'`, `'masters'`, `'phd'`, `'self_taught'` |
| `tech_background`   | Enum      | User's primary technical background                                      | Required, one of `'software_engineer'`, `'hardware_engineer'`, `'student'`, `'hobbyist'` |
| `primary_goal`      | Enum      | Main objective for learning/using the textbook                           | Required, one of `'career_switch'`, `'academic'`, `'hobby_project'`, `'startup_founder'` |
| `learning_mode`     | Enum      | Preferred mode of consuming information                                  | Required, one of `'visual'`, `'textual'`, `'code_first'` |
| `learning_speed`    | Enum      | Preferred pace or depth of learning                                      | Required, one of `'intensive'`, `'balanced'`, `'casual'` |
| `time_per_week`     | Integer   | Estimated hours per week available for learning                          | Required, positive integer (e.g., 5, 10, 20)         |
| `preferred_language`| String    | User's preferred language for content display                            | Required, two-letter language code (e.g., `'en'`, `'ur'`), default `'en'` |

### Relationships

-   **One-to-One**: `User Profile` to `auth.users` (Supabase). Each user has one associated profile.

### Validation

-   All enum fields will be strictly validated using Zod.
-   `time_per_week` will be validated to be a positive integer.
-   `preferred_language` will default to 'en' and be validated against a list of supported language codes.
