# Quickstart: Dark Mode Landing Page

**Feature**: 009-dark-mode-landing-page
**Date**: 2025-12-12

## Prerequisites

- Node.js 20+ installed
- Project cloned and on branch `009-dark-mode-landing-page`
- Website dependencies installed (`cd website && npm install`)

## Setup

### 1. Install Framer Motion

```bash
cd website
npm install framer-motion
```

### 2. Create Component Directory

```bash
mkdir -p src/components/landing
mkdir -p static/img/landing
mkdir -p tests/components/landing
```

### 3. Verify Tailwind Configuration

The existing `tailwind.config.js` already has:
- Dark mode support (`darkMode: ["class", '[data-theme="dark"]']`)
- Content paths including `src/**/*.{ts,tsx}`
- `tailwindcss-animate` plugin
- `preflight: false` (prevents Docusaurus conflicts)

No changes required.

## Development Workflow

### Start Development Server

```bash
npm start
```

Visit `http://localhost:3000` to see the landing page.

### Run Tests

```bash
# Unit tests
npm test

# E2E tests (Playwright)
npx playwright test
```

### Build for Production

```bash
npm run build
npm run serve  # Preview production build
```

## Component Development Order

Follow this order to maintain testability and incremental progress:

### Phase 1: Foundation Components
1. `AnimatedSection.tsx` - Reusable scroll animation wrapper
2. `GlowButton.tsx` - Primary and secondary CTA buttons

### Phase 2: Main Sections (P1 Priority)
3. `HeroSection.tsx` - Hero with headline, subheadline, CTAs
4. `CurriculumCard.tsx` - Individual curriculum card
5. `CurriculumSection.tsx` - Grid of curriculum cards
6. `FooterCTA.tsx` - Final call-to-action section

### Phase 3: Feature Demos (P2 Priority)
7. `ChatbotDemo.tsx` - Animated mock chat window
8. `PersonalizationDemo.tsx` - Toggle animation demo
9. `FeatureDemo.tsx` - Container for feature demos
10. `FeaturesSection.tsx` - Zig-zag layout wrapper

### Phase 4: Hardware Section (P2 Priority)
11. `HardwareItem.tsx` - Individual hardware item
12. `LabSection.tsx` - Loadout-style hardware display

### Phase 5: Integration
13. Update `src/pages/index.tsx` - Compose all sections

## Quick Component Template

```tsx
// src/components/landing/ExampleSection.tsx
import { motion } from 'framer-motion';

interface ExampleSectionProps {
  // props
}

export function ExampleSection({ }: ExampleSectionProps) {
  return (
    <motion.section
      initial={{ opacity: 0, y: 20 }}
      whileInView={{ opacity: 1, y: 0 }}
      transition={{ duration: 0.5 }}
      viewport={{ once: true }}
      className="py-20 px-4"
    >
      {/* Section content */}
    </motion.section>
  );
}
```

## Quick Test Template

```tsx
// tests/components/landing/ExampleSection.test.tsx
import { render, screen } from '@testing-library/react';
import { ExampleSection } from '@site/src/components/landing/ExampleSection';

describe('ExampleSection', () => {
  it('renders correctly', () => {
    render(<ExampleSection />);
    // assertions
  });
});
```

## Key Tailwind Classes Reference

| Purpose | Classes |
|---------|---------|
| Deep void background | `bg-slate-950` |
| Primary text | `text-slate-50` |
| Muted text | `text-slate-400` |
| Glassmorphism card | `bg-slate-900/50 backdrop-blur-md border border-white/10` |
| Cyan accent | `text-cyan-400` or `bg-cyan-400` |
| Violet accent | `text-violet-600` or `bg-violet-600` |
| Gradient text | `bg-gradient-to-r from-cyan-400 to-violet-600 bg-clip-text text-transparent` |
| Glow effect | `shadow-[0_0_20px_rgba(34,211,238,0.5)]` |
| Gradient background (footer) | `bg-gradient-to-t from-violet-900/20` |

## Troubleshooting

### Animations not working
- Ensure Framer Motion is installed
- Check that `motion` components are properly imported
- Verify `viewport={{ once: true }}` is set

### Styles not applying
- Run `npm run clear` to clear Docusaurus cache
- Check Tailwind content paths in config
- Ensure classes are not being purged in production

### Hydration mismatch errors
- Use `viewport={{ once: true }}` on animations
- Avoid conditional rendering based on window/document
- Use `suppressHydrationWarning` if needed for dynamic content

## Files to Modify

| File | Action |
|------|--------|
| `website/package.json` | Add framer-motion dependency |
| `website/src/pages/index.tsx` | Replace with new landing page |
| `website/src/pages/index.module.css` | Delete (Tailwind replaces) |
| `website/src/components/landing/*` | Create all new components |
| `website/tests/components/landing/*` | Create component tests |
| `website/static/img/landing/*` | Add robot hero image |
