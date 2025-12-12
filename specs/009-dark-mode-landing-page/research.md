# Research: Dark Mode Landing Page

**Feature**: 009-dark-mode-landing-page
**Date**: 2025-12-12
**Status**: Complete

## Research Tasks

### RT-1: Framer Motion Integration with Docusaurus

**Question**: How to properly integrate Framer Motion in a Docusaurus 3.x project?

**Decision**: Standard npm installation with SSR considerations
**Rationale**:
- Framer Motion works with React 19 (verified via compatibility docs)
- Docusaurus handles SSR; Framer Motion's `LazyMotion` can reduce bundle size
- Use `domAnimation` feature set for basic scroll animations

**Implementation**:
```bash
npm install framer-motion
```

**Key Patterns**:
```tsx
import { motion } from 'framer-motion';

// Scroll-triggered animation
<motion.div
  initial={{ opacity: 0, y: 20 }}
  whileInView={{ opacity: 1, y: 0 }}
  transition={{ duration: 0.5 }}
  viewport={{ once: true }}
>
  {children}
</motion.div>
```

**Alternatives Considered**:
- `react-intersection-observer` + CSS: More manual, less smooth
- AOS (Animate on Scroll): jQuery-era library, not React-native

---

### RT-2: Glassmorphism Effect in Tailwind

**Question**: Best approach for glassmorphism cards with Tailwind CSS?

**Decision**: Use Tailwind's built-in utilities with custom opacity values
**Rationale**:
- Tailwind 3.x has native `backdrop-blur` support
- Custom colors can be defined inline or in config
- Spec provides exact classes: `bg-slate-900/50 backdrop-blur-md border border-white/10`

**Implementation**:
```tsx
// Glassmorphism card
<div className="bg-slate-900/50 backdrop-blur-md border border-white/10 rounded-xl p-6">
  {content}
</div>
```

**Browser Support**:
- `backdrop-filter` supported in all modern browsers
- Safari requires `-webkit-backdrop-filter` (Tailwind handles this)
- Fallback: solid background for unsupported browsers (auto via CSS cascade)

---

### RT-3: Gradient Text Effect

**Question**: How to implement cyan-to-violet gradient on text "Metal"?

**Decision**: Use Tailwind's background-clip text utilities
**Rationale**:
- Native Tailwind support for gradient text
- Spec colors: cyan (#22d3ee) to violet (#7c3aed)

**Implementation**:
```tsx
<span className="bg-gradient-to-r from-cyan-400 to-violet-600 bg-clip-text text-transparent">
  Metal
</span>
```

**Note**: `text-transparent` required for gradient to show through text

---

### RT-4: Glow Effect on Buttons

**Question**: How to achieve subtle glow effect on buttons?

**Decision**: Use Tailwind's drop-shadow with custom color
**Rationale**:
- `drop-shadow` can accept color values
- Better than `box-shadow` for glow effects
- Can be combined with hover states

**Implementation**:
```tsx
// Primary CTA with glow
<button className="
  bg-cyan-400 text-slate-950 font-semibold px-8 py-3 rounded-lg
  shadow-[0_0_20px_rgba(34,211,238,0.5)]
  hover:shadow-[0_0_30px_rgba(34,211,238,0.7)]
  transition-shadow duration-300
">
  Start Reading
</button>

// Secondary CTA (outlined)
<button className="
  bg-transparent border-2 border-white text-white px-8 py-3 rounded-lg
  hover:bg-white/10
  transition-colors duration-300
">
  Watch Demo
</button>
```

---

### RT-5: Reduced Motion Accessibility

**Question**: How to respect user's reduced motion preferences?

**Decision**: Use Framer Motion's `useReducedMotion` hook
**Rationale**:
- Built-in hook detects `prefers-reduced-motion` media query
- Easy to conditionally disable/reduce animations
- Aligns with accessibility requirements (SC-009)

**Implementation**:
```tsx
import { motion, useReducedMotion } from 'framer-motion';

function AnimatedSection({ children }) {
  const shouldReduceMotion = useReducedMotion();

  return (
    <motion.div
      initial={shouldReduceMotion ? false : { opacity: 0, y: 20 }}
      whileInView={{ opacity: 1, y: 0 }}
      transition={{ duration: shouldReduceMotion ? 0 : 0.5 }}
    >
      {children}
    </motion.div>
  );
}
```

---

### RT-6: Docusaurus Layout Override

**Question**: How to create a custom landing page while maintaining Docusaurus navigation?

**Decision**: Use `Layout` component wrapper with custom content
**Rationale**:
- Existing `index.tsx` already uses `Layout` from `@theme/Layout`
- Can completely replace `HomepageHeader` with custom sections
- Preserves navbar, footer, and theme context

**Implementation**:
```tsx
import Layout from '@theme/Layout';

export default function Home() {
  return (
    <Layout title="Physical AI & Robotics" description="...">
      <main className="bg-slate-950 min-h-screen">
        <HeroSection />
        <CurriculumSection />
        <FeaturesSection />
        <LabSection />
        <FooterCTA />
      </main>
    </Layout>
  );
}
```

**Note**: Docusaurus's default footer may need to be hidden/overridden since spec includes custom footer

---

### RT-7: Lucide Icons for Curriculum Cards

**Question**: Which Lucide icons map to the spec's requirements?

**Decision**: Use these specific icons
**Rationale**: Lucide React (v0.556.0) already installed in project

**Icon Mapping**:
| Section | Spec Description | Lucide Icon | Import |
|---------|-----------------|-------------|--------|
| Nervous System | Network Graph | `Network` | `lucide-react` |
| Digital Twin | 3D Cube | `Box` or `Cuboid` | `lucide-react` |
| Brain | AI Chip | `Cpu` or `BrainCircuit` | `lucide-react` |
| Body | Robot Walking | `PersonStanding` or `Bot` | `lucide-react` |

---

### RT-8: Image Handling for Robot Hero

**Question**: Best approach for hero robot image?

**Decision**: Use Docusaurus static assets with placeholder
**Rationale**:
- `/static/img/` is served directly
- Can use SVG for wireframe style (smaller, scalable)
- Placeholder until final asset provided

**Implementation**:
```tsx
// Option A: Real image
<img
  src="/img/landing/robot-hero.png"
  alt="Unitree G1 humanoid robot"
  className="w-full max-w-md"
/>

// Option B: Placeholder (development)
<div className="w-full max-w-md h-96 bg-slate-800/50 rounded-2xl flex items-center justify-center">
  <Bot className="w-24 h-24 text-cyan-400/50" />
</div>
```

---

## Tailwind Configuration Updates

**Required additions to `tailwind.config.js`**:

```javascript
// Extend existing colors for spec palette
colors: {
  // Existing colors...
  'deep-void': '#020617',    // bg-slate-950
  'electric-cyan': '#22d3ee', // cyan-400
  'deep-violet': '#7c3aed',   // violet-600
}
```

**Note**: These colors already exist in Tailwind's default palette (slate-950, cyan-400, violet-600), so no config changes needed.

---

## Dependencies to Install

```bash
cd website
npm install framer-motion
```

No other dependencies required - all others already exist in project.

---

## Risk Mitigations

| Risk | Mitigation |
|------|------------|
| Tailwind/Docusaurus style conflicts | `preflight: false` already set in config |
| SSR hydration mismatch with animations | Use `viewport={{ once: true }}` and `initial={{ opacity: 0 }}` |
| Large hero image slowing page load | Use optimized WebP/AVIF, lazy loading |
| Accessibility issues | Built-in reduced motion support, semantic HTML, ARIA labels |
