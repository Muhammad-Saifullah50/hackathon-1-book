# Physical AI & Humanoid Robotics - Learning Platform Frontend

> A modern, interactive learning platform built with Docusaurus 3.9, React 19, and TypeScript 5.6

[![Docusaurus](https://img.shields.io/badge/Docusaurus-3.9.2-green.svg)](https://docusaurus.io/)
[![React](https://img.shields.io/badge/React-19.0.0-blue.svg)](https://reactjs.org/)
[![TypeScript](https://img.shields.io/badge/TypeScript-5.6.2-blue.svg)](https://www.typescriptlang.org/)
[![Tailwind CSS](https://img.shields.io/badge/Tailwind-3.4.18-38B2AC.svg)](https://tailwindcss.com/)
[![Node](https://img.shields.io/badge/Node-%3E%3D20.0-green.svg)](https://nodejs.org/)

---

## Overview

This is the frontend application for the **Physical AI & Humanoid Robotics** educational platform - a world-class technical textbook that bridges digital AI with embodied intelligence. Built on Docusaurus, it delivers an engaging learning experience with interactive features, personalized content, and an AI-powered RAG tutor.

### Key Features

- **Rich Content Delivery**: 4 modules, 13 weeks of comprehensive robotics curriculum
- **Interactive Learning**: AI-powered chatbot tutor with RAG capabilities
- **Personalization Engine**: Adaptive content based on learner profiles
- **Multilingual Support**: Urdu translation with expansion planned
- **Modern Authentication**: Better Auth integration with JWT tokens
- **Dark Mode**: Seamless theme switching with preference persistence
- **Responsive Design**: Mobile-first approach with Tailwind CSS
- **Accessible Components**: Built with Radix UI primitives
- **Smooth Animations**: Framer Motion for engaging transitions

---

## Technology Stack

| Category | Technology | Version | Purpose |
|----------|-----------|---------|---------|
| **Framework** | Docusaurus | 3.9.2 | Static site generator with React |
| **UI Library** | React | 19.0.0 | Component-based UI framework |
| **Language** | TypeScript | 5.6.2 | Type-safe development |
| **Styling** | Tailwind CSS | 3.4.18 | Utility-first CSS framework |
| **Components** | Radix UI | 2.2.6+ | Accessible UI primitives |
| **Animation** | Framer Motion | 12.23.26 | Motion library for React |
| **Validation** | Zod | 4.1.13 | Schema validation |
| **Auth** | Better Auth | 1.2.8 | Modern authentication |
| **AI Chat** | OpenAI ChatKit | 1.3.0 | Chat interface components |
| **Icons** | Lucide React | 0.556.0 | Icon library |
| **Testing** | Jest | 30.2.0 | Unit testing framework |
| **E2E Testing** | Playwright | 1.57.0 | Browser automation |

---

## Project Structure

```
website/
├── docs/                          # Markdown curriculum content
│   ├── module-01/                 # Foundations of Physical AI
│   ├── module-02/                 # Digital Twin & Simulation
│   ├── module-03/                 # AI-Robot Brain Integration
│   └── module-04/                 # Vision-Language-Action Models
├── src/
│   ├── components/                # React components
│   │   ├── auth/                  # Authentication components
│   │   │   ├── AuthNavbarItems.tsx
│   │   │   ├── LoginForm.tsx
│   │   │   └── SignupForm.tsx
│   │   ├── landing/               # Landing page sections
│   │   │   ├── HeroSection.tsx
│   │   │   ├── FeaturesSection.tsx
│   │   │   ├── LabSection.tsx
│   │   │   ├── ChatbotDemo.tsx
│   │   │   ├── PersonalizationDemo.tsx
│   │   │   └── AnimatedSection.tsx
│   │   ├── profile/               # User profile components
│   │   │   ├── ProfileWizardStep1.tsx
│   │   │   ├── ProfileWizardStep2.tsx
│   │   │   ├── ProfileWizardStep3.tsx
│   │   │   └── PersonalizationBar.tsx
│   │   ├── translation/           # Translation UI
│   │   │   └── TranslationBar.tsx
│   │   ├── ui/                    # Reusable UI components
│   │   │   ├── select.tsx
│   │   │   ├── sheet.tsx
│   │   │   ├── card.tsx
│   │   │   ├── badge.tsx
│   │   │   ├── alert.tsx
│   │   │   ├── table.tsx
│   │   │   ├── Logo.tsx
│   │   │   └── image.tsx
│   │   ├── ChatWidget.tsx         # AI tutor chat interface
│   │   ├── SelectionPopup.tsx     # Text selection features
│   │   └── SimToRealWarning.tsx   # Safety warnings
│   ├── hooks/                     # Custom React hooks
│   │   ├── useAuth.tsx            # Authentication logic
│   │   ├── useProfile.tsx         # Profile management
│   │   ├── usePersonalization.ts  # Content personalization
│   │   ├── useTranslation.ts      # Translation features
│   │   └── useSafeColorMode.ts    # Theme management
│   ├── pages/                     # Custom pages
│   │   ├── index.tsx              # Landing page
│   │   ├── login.tsx              # Login page
│   │   ├── signup.tsx             # Signup page
│   │   └── signup-wizard/         # Onboarding wizard
│   ├── theme/                     # Docusaurus theme overrides
│   │   ├── CodeBlock/             # Enhanced code blocks
│   │   ├── DocPaginator/          # Document navigation
│   │   ├── DocSidebar/            # Sidebar customization
│   │   ├── Heading/               # Custom headings
│   │   ├── Layout/                # Page layout wrapper
│   │   └── NavbarItem/            # Navigation items
│   ├── types/                     # TypeScript type definitions
│   ├── services/                  # API service layer
│   ├── utils/                     # Utility functions
│   ├── css/                       # Global styles
│   └── data/                      # Static data files
├── static/                        # Static assets (images, files)
├── tests/                         # Test files
│   └── unit/                      # Unit tests
├── docusaurus.config.ts           # Docusaurus configuration
├── tailwind.config.js             # Tailwind CSS configuration
├── jest.config.js                 # Jest test configuration
├── tsconfig.json                  # TypeScript configuration
├── sidebars.ts                    # Documentation sidebar
└── package.json                   # Dependencies and scripts
```

---

## Prerequisites

Ensure you have the following installed:

- **Node.js**: >= 20.0
- **npm**: >= 9.0 (comes with Node.js)
- **Backend Server**: FastAPI backend running on port 8000 (see `../backend/README.md`)

---

## Installation

### 1. Clone the Repository

```bash
git clone <repository-url>
cd hackathon-1-book/website
```

### 2. Install Dependencies

```bash
npm install
```

This will install all required dependencies including:
- Docusaurus core and presets
- React 19 and React DOM
- TypeScript and type definitions
- Tailwind CSS and plugins
- Testing libraries (Jest, Playwright)
- UI component libraries (Radix UI)
- Additional utilities (Zod, Framer Motion, etc.)

### 3. Environment Configuration

Create a `.env` file in the `website/` directory:

```bash
# Backend API Configuration
NEXT_PUBLIC_BACKEND_URL=http://localhost:8000

# Authentication Configuration
AUTH_URL=http://localhost:8000/api/auth
DOMAIN_KEY=your-domain-key-here

# Optional: Analytics, etc.
# GA_TRACKING_ID=UA-XXXXX-Y
```

**Note**: The `.env` file is gitignored. Never commit sensitive credentials.

---

## Development

### Start Development Server

```bash
npm start
```

This command starts a local development server at `http://localhost:3000` with hot reloading. Changes to source files will automatically refresh the browser.

### Build for Production

```bash
npm run build
```

Generates static content into the `build` directory. The output is optimized and ready for deployment.

### Serve Production Build Locally

```bash
npm run serve
```

Serves the production build locally to test before deployment.

### Type Checking

```bash
npm run typecheck
```

Runs TypeScript compiler to check for type errors without emitting files.

### Clear Cache

```bash
npm run clear
```

Clears the Docusaurus cache, generated assets, and build artifacts.

---

## Available Scripts

| Command | Description |
|---------|-------------|
| `npm start` | Start development server on port 3000 |
| `npm run build` | Build production bundle |
| `npm run serve` | Serve production build locally |
| `npm test` | Run Jest unit tests |
| `npm run test:watch` | Run tests in watch mode |
| `npm run typecheck` | Type-check without emitting files |
| `npm run clear` | Clear Docusaurus cache |
| `npm run swizzle` | Eject Docusaurus components for customization |
| `npm run write-translations` | Extract translatable strings |
| `npm run write-heading-ids` | Generate heading IDs for docs |

---

## Testing

### Unit Tests (Jest)

The project uses Jest with React Testing Library for unit and integration tests.

**Run all tests:**
```bash
npm test
```

**Run tests in watch mode:**
```bash
npm run test:watch
```

**Test file locations:**
- Component tests: `src/**/*.{test,spec}.{ts,tsx}`
- Unit tests: `tests/unit/**/*.{test,spec}.{ts,tsx}`

**Test setup:**
- Configuration: `jest.config.js`
- Setup file: `setupTests.ts`
- Mocks: `tests/unit/mocks/`

### E2E Tests (Playwright)

**Note**: Playwright configuration is currently in development. The test runner will be available for multi-browser E2E testing (Chromium, Firefox, WebKit).

Expected command structure:
```bash
npm run test:e2e           # Run E2E tests
npm run test:e2e:headed    # Run with browser UI
npm run test:e2e:debug     # Debug mode
```

---

## Component Inventory

### Authentication Components

| Component | Path | Description |
|-----------|------|-------------|
| `AuthNavbarItems` | `components/auth/` | Navbar login/signup buttons |
| `LoginForm` | `components/auth/` | Email/password login form |
| `SignupForm` | `components/auth/` | User registration form |

### Profile & Personalization

| Component | Path | Description |
|-----------|------|-------------|
| `ProfileWizardStep1` | `components/profile/` | Age, education, background |
| `ProfileWizardStep2` | `components/profile/` | Learning preferences |
| `ProfileWizardStep3` | `components/profile/` | Goals and commitment |
| `PersonalizationBar` | `components/profile/` | Content personalization controls |

### Landing Page

| Component | Path | Description |
|-----------|------|-------------|
| `HeroSection` | `components/landing/` | Main hero with CTA |
| `FeaturesSection` | `components/landing/` | Key platform features |
| `LabSection` | `components/landing/` | Hardware lab showcase |
| `ChatbotDemo` | `components/landing/` | Interactive AI demo |
| `PersonalizationDemo` | `components/landing/` | Personalization showcase |
| `AnimatedSection` | `components/landing/` | Animated content wrapper |
| `CurriculumSection` | `components/landing/` | Course curriculum overview |
| `FooterCTA` | `components/landing/` | Final call-to-action |

### UI Components (Radix-based)

| Component | Path | Description |
|-----------|------|-------------|
| `Select` | `components/ui/select.tsx` | Accessible dropdown select |
| `Sheet` | `components/ui/sheet.tsx` | Slide-out panel |
| `Card` | `components/ui/card.tsx` | Content card container |
| `Badge` | `components/ui/badge.tsx` | Status/label badge |
| `Alert` | `components/ui/alert.tsx` | Alert/notification box |
| `Table` | `components/ui/table.tsx` | Data table components |
| `Logo` | `components/ui/Logo.tsx` | Application logo |

### Theme Overrides

| Component | Path | Description |
|-----------|------|-------------|
| `Heading` | `theme/Heading/` | Enhanced heading with personalization |
| `Layout` | `theme/Layout/` | Page layout with chat widget |
| `NavbarItem` | `theme/NavbarItem/` | Custom navbar items (auth) |
| `CodeBlock` | `theme/CodeBlock/` | Syntax-highlighted code blocks |
| `DocSidebar` | `theme/DocSidebar/` | Documentation sidebar |
| `DocPaginator` | `theme/DocPaginator/` | Prev/next navigation |

---

## Custom Hooks

### `useAuth`

Manages authentication state and operations.

```typescript
const { user, isLoading, login, signup, logout } = useAuth();

// Login
await login({ email, password });

// Signup
await signup({ email, password });

// Logout
await logout();
```

### `useProfile`

Handles user profile data and wizard state.

```typescript
const { profile, updateProfile, isLoading } = useProfile();

await updateProfile({
  age_range: "18_24",
  education_level: "bachelors",
  tech_background: "intermediate"
});
```

### `usePersonalization`

Manages content personalization features.

```typescript
const {
  personalizeContent,
  isLoading,
  remainingQuota
} = usePersonalization();

const result = await personalizeContent({
  content: "Complex robotics text...",
  userProfile: profile
});
```

### `useTranslation`

Handles content translation to supported languages.

```typescript
const {
  translateContent,
  isLoading,
  remainingQuota
} = useTranslation();

const translated = await translateContent({
  content: "English text",
  targetLanguage: "ur"
});
```

### `useSafeColorMode`

Safely manages theme/color mode with SSR compatibility.

```typescript
const { colorMode, setColorMode } = useSafeColorMode();

// Toggle theme
setColorMode(colorMode === "dark" ? "light" : "dark");
```

---

## Styling with Tailwind CSS

This project uses Tailwind CSS with a custom theme configuration.

### Color System

The theme uses CSS variables for dynamic theming:

```css
/* Light mode */
--background: 0 0% 100%;
--foreground: 222.2 84% 4.9%;
--primary: 222.2 47.4% 11.2%;
--secondary: 210 40% 96.1%;

/* Dark mode */
[data-theme="dark"] {
  --background: 222.2 84% 4.9%;
  --foreground: 210 40% 98%;
  --primary: 210 40% 98%;
  --secondary: 217.2 32.6% 17.5%;
}
```

### Using Tailwind in Components

```tsx
import { cn } from "@/lib/utils";

export function MyComponent() {
  return (
    <div className={cn(
      "rounded-lg border bg-card p-6",
      "hover:shadow-lg transition-shadow"
    )}>
      <h2 className="text-2xl font-bold text-foreground">
        Content
      </h2>
    </div>
  );
}
```

### Important Tailwind Notes

- **Preflight disabled**: `corePlugins.preflight: false` to avoid conflicts with Docusaurus styles
- **Dark mode**: Uses both class and data-attribute strategy
- **Animations**: Includes tailwindcss-animate plugin for smooth transitions

---

## API Integration

### Backend Configuration

The frontend communicates with the FastAPI backend via:

```typescript
// Configured in docusaurus.config.ts
const backendUrl = process.env.NEXT_PUBLIC_BACKEND_URL || 'http://localhost:8000';
```

### Authentication Flow

1. User submits credentials via `LoginForm` or `SignupForm`
2. `useAuth` hook sends request to `/auth/login` or `/auth/signup`
3. Backend returns JWT tokens (access + refresh)
4. Tokens stored in httpOnly cookies
5. All subsequent requests include credentials

### API Endpoints Used

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/auth/signup` | POST | User registration |
| `/auth/login` | POST | User authentication |
| `/auth/logout` | POST | Session termination |
| `/auth/me` | GET | Current user info |
| `/chatkit` | POST | AI tutor interactions |
| `/personalize` | POST | Content personalization |
| `/translate` | POST | Content translation |

---

## Deployment

### Build Production Bundle

```bash
npm run build
```

The `build/` directory contains:
- Optimized static HTML, CSS, JS
- Pre-rendered pages for fast loading
- Asset fingerprinting for cache busting

### Deployment Platforms

**Vercel** (Recommended):
```bash
npm install -g vercel
vercel --prod
```

**Netlify**:
```bash
npm run build
# Deploy build/ directory via Netlify CLI or dashboard
```

**Static Hosting**:
Upload the `build/` directory to any static host (S3, GitHub Pages, etc.)

### Environment Variables (Production)

Set these in your deployment platform:

```bash
NEXT_PUBLIC_BACKEND_URL=https://api.yourplatform.com
AUTH_URL=https://api.yourplatform.com/api/auth
DOMAIN_KEY=production-domain-key
```

### Backend Requirements

Ensure the FastAPI backend is:
- Deployed and accessible at `NEXT_PUBLIC_BACKEND_URL`
- CORS configured to allow requests from frontend domain
- Cookies set with appropriate `SameSite` and `Secure` flags

---

## Architecture Decisions

### Why Docusaurus?

- **Documentation-first**: Built for technical content with excellent MDX support
- **Performance**: Static site generation for fast load times
- **SEO-friendly**: Pre-rendered HTML for search engines
- **Extensibility**: Plugin system for custom features
- **Community**: Strong ecosystem and maintained by Facebook

### Why React 19?

- **Latest features**: Access to newest React capabilities
- **Performance**: Improved rendering and concurrent features
- **Future-proof**: Aligned with React's direction

### Why Tailwind CSS?

- **Rapid development**: Utility-first approach speeds up styling
- **Consistency**: Design system enforced through configuration
- **Dark mode**: First-class support for theme switching
- **Bundle size**: Purges unused styles in production

### Why Radix UI?

- **Accessibility**: WCAG compliant out of the box
- **Unstyled**: Full control over styling with Tailwind
- **Composable**: Build complex components from primitives
- **Keyboard navigation**: Built-in keyboard support

### Why Better Auth?

- **Modern**: JWT-based authentication
- **Flexible**: Works with any database
- **Secure**: Industry-standard security practices
- **Developer-friendly**: Simple API and good documentation

---

## Troubleshooting

### Issue: "Module not found" errors

**Solution**: Clear cache and reinstall dependencies
```bash
npm run clear
rm -rf node_modules package-lock.json
npm install
```

### Issue: TypeScript errors in node_modules

**Solution**: Ensure `skipLibCheck: true` in `tsconfig.json`
```json
{
  "compilerOptions": {
    "skipLibCheck": true
  }
}
```

### Issue: Tailwind styles not applied

**Solution**: Check that your component files are covered by `content` in `tailwind.config.js`
```javascript
content: [
  './src/**/*.{ts,tsx}',
  './docs/**/*.{md,mdx}',
],
```

### Issue: Dark mode not working

**Solution**: Use `useSafeColorMode` hook instead of direct `useColorMode`
```typescript
import { useSafeColorMode } from '@/hooks/useSafeColorMode';
```

### Issue: Backend API calls failing

**Solution**:
1. Verify backend is running on port 8000
2. Check CORS configuration in backend
3. Ensure `NEXT_PUBLIC_BACKEND_URL` is set correctly
4. Check browser console for specific errors

### Issue: Jest tests failing with import errors

**Solution**: Check `moduleNameMapper` in `jest.config.js` matches your imports
```javascript
moduleNameMapper: {
  '^@/(.*)$': '<rootDir>/src/$1',
  '^@site/(.*)$': '<rootDir>/$1',
}
```

---

## Contributing

This project follows Test-Driven Development (TDD) principles:

### Development Workflow

1. **Red**: Write a failing test first
2. **Green**: Implement minimum code to pass
3. **Refactor**: Clean up while keeping tests green

### Code Standards

**TypeScript**:
- Use functional components with hooks
- Type everything with interfaces/types
- No `any` types (use `unknown` if necessary)

**React**:
- One component per file
- Use meaningful component and variable names
- Destructure props at function signature

**Tailwind**:
- Use utility classes, avoid custom CSS
- Follow mobile-first responsive design
- Use theme colors from config

**Testing**:
- Test user behavior, not implementation
- Use React Testing Library best practices
- Mock external dependencies

### Adding New Features

1. Create/update feature spec in `../specs/`
2. Write tests first (TDD)
3. Implement feature
4. Update documentation
5. Create PR with clear description

---

## Project Philosophy

This platform embodies five core principles:

1. **Physics First**: Robotics is physical; code must respect real-world constraints
2. **Sim-to-Real**: Design → Simulation → Real Deployment workflow
3. **Safety**: Always emphasize safety protocols in robotics code
4. **TDD Mandate**: All features follow Red-Green-Refactor cycle
5. **Clarity**: Complex concepts (SLAM, VLA, Kinematics) explained step-by-step

---

## Learning Resources

### Docusaurus
- [Official Documentation](https://docusaurus.io/)
- [API Reference](https://docusaurus.io/docs/api/docusaurus-config)
- [Swizzling Guide](https://docusaurus.io/docs/swizzling)

### React 19
- [React Docs](https://react.dev/)
- [React 19 Release Notes](https://react.dev/blog/2024/12/05/react-19)

### Tailwind CSS
- [Documentation](https://tailwindcss.com/docs)
- [Utility Reference](https://tailwindcss.com/docs/utility-first)

### Radix UI
- [Primitives Documentation](https://www.radix-ui.com/primitives)
- [Accessibility Guide](https://www.radix-ui.com/primitives/docs/overview/accessibility)

---

## License

[Specify your license here - e.g., MIT, Apache 2.0, etc.]

---

## Support

For questions, issues, or contributions:

- **Issues**: [GitHub Issues](link-to-issues)
- **Discussions**: [GitHub Discussions](link-to-discussions)
- **Email**: support@yourplatform.com

---

## Acknowledgments

Built with love for the robotics and AI community. Special thanks to:

- Docusaurus team for the amazing framework
- React team for continuous innovation
- Radix UI for accessible components
- Tailwind CSS for the utility-first revolution
- OpenAI for AI integration capabilities

---

**Happy Learning! Welcome to the future of Physical AI and Humanoid Robotics.**
