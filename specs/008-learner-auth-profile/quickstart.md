# Quickstart: Authentication & Detailed Profiling

This guide provides the minimal steps to get the Authentication and Detailed Learner Profiling feature up and running locally.

## Prerequisites

- Node.js (for Docusaurus frontend)
- Python (for backend services)
- Git
- Docker (for local Supabase setup, or access to a remote Supabase project)

## 1. Setup Supabase

This feature relies on Supabase for user authentication and profile storage.

### Option A: Local Supabase (Recommended for development)

1.  **Install Supabase CLI**: Follow the official Supabase CLI installation guide.
2.  **Start Supabase**:
    ```bash
    supabase init
    supabase start
    ```
3.  **Apply Migrations**: This feature requires a `profiles` table to store extended user data. You will need to create a migration file with the schema defined in `data-model.md` and apply it.
    ```bash
    # Example: Create a new migration for profiles table
    supabase migration new create_profiles_table
    # Edit the generated SQL file in supabase/migrations/ to add the profiles table schema.
    # The table should link to auth.users via a foreign key user_id.
    supabase db push
    ```
    *Note*: Ensure your `profiles` table schema matches the `UserProfile` entity defined in `data-model.md`.

### Option B: Remote Supabase Project

1.  **Create Project**: Go to [Supabase](https://supabase.com/) and create a new project.
2.  **Get Credentials**: Note your Project URL and `anon` public key (found in Project Settings -> API).
3.  **Apply Schema**: Manually create the `profiles` table and any necessary RLS policies in your Supabase project as defined in `data-model.md`.

## 2. Configure Environment Variables

Create or update `.env` files for both your backend and frontend.

### Backend (`backend/.env`)

```
SUPABASE_URL="YOUR_SUPABASE_PROJECT_URL"
SUPABASE_KEY="YOUR_SUPABASE_ANON_KEY"
BETTER_AUTH_SECRET="A_STRONG_SECRET_FOR_AUTH" # Placeholder, better-auth specifics needed
```

### Frontend (`website/.env`)

```
NEXT_PUBLIC_SUPABASE_URL="YOUR_SUPABASE_PROJECT_URL"
NEXT_PUBLIC_SUPABASE_ANON_KEY="YOUR_SUPABASE_ANON_KEY"
```

## 3. Run Backend Services

Navigate to the `backend/` directory and start the Python services.

```bash
cd backend/
pip install -r requirements.txt # Or use poetry/uv if configured
python main.py # Or equivalent command to start your FastAPI/Flask app
```

## 4. Run Frontend (Docusaurus)

Navigate to the `website/` directory and start the Docusaurus development server.

```bash
cd website/
npm install # Or yarn install
npm start # Or yarn start
```

## 5. Access the Feature

-   Open your browser to the Docusaurus development server URL (usually `http://localhost:3000`).
-   Navigate to the signup page (e.g., `/signup-wizard` as defined in `plan.md`).
-   Test the multi-step learner profile wizard and the personalization bar.
