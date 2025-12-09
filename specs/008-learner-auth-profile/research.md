# Research for Authentication & Detailed Profiling

## Decisions & Rationale

This document outlines the research conducted for the "Authentication & Detailed Profiling" feature, aiming to clarify technical approaches and best practices for its implementation.

### 1. `better-auth` Integration with Supabase

- **Decision**: Integrate `better-auth` for authentication, leveraging Supabase as the backend for user and profile data. `better-auth` will handle user registration, login, and session management, while Supabase's PostgreSQL database will store the extended user profiles.
- **Rationale**: The user explicitly requested `better-auth` and Supabase. Research will focus on the seamless integration patterns and schema extensions required to accommodate the detailed learner profile.
- **Alternatives Considered**: None, as the core technologies were specified.

### 2. `Shadcn UI` for Multi-step Forms

- **Decision**: Utilize `Shadcn UI` components to build the multi-step "Learner Profile" wizard.
- **Rationale**: `Shadcn UI` is a component library built on top of Tailwind CSS and Radix UI, known for its accessibility, customizability, and modern aesthetic, aligning with the project's visual and UI standards. Research will focus on implementing a fluid, multi-step form experience with validation.
- **Alternatives Considered**: None, as the UI component library was specified.

### 3. `Zod` Schema Validation

- **Decision**: Implement `Zod` for schema validation, particularly for the enum fields within the learner profile. This will be applied both on the frontend (for immediate user feedback) and on the backend (for data integrity before saving to Supabase).
- **Rationale**: `Zod` provides a robust, TypeScript-first schema declaration and validation library. Its ability to infer static types from schemas will enhance developer experience and reduce runtime errors. Research will cover defining complex enum schemas and integrating with API endpoints.
- **Alternatives Considered**: Yup, Joi (rejected due to preference for TypeScript-native solution).

### 4. Handling User Profile Data Privacy

- **Decision**: Implement strong privacy safeguards for user profile data, aligning with the "Privacy: Explicitly state that this data is used *only* for tailoring the textbook experience" rule.
- **Rationale**: Protecting user data and ensuring transparency in its use is critical for trust and compliance. Research will involve reviewing best practices for data storage, access control within Supabase, and communicating privacy policies effectively within the application.
- **Alternatives Considered**: None, privacy is a non-negotiable requirement.

## Unresolved Questions / Further Research Needed

*(No `NEEDS CLARIFICATION` markers were present in the spec, but detailed research into the integration specifics will be ongoing during the planning phase.)*

---
