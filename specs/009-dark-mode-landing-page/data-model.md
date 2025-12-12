# Data Model: Dark Mode Landing Page

**Feature**: 009-dark-mode-landing-page
**Date**: 2025-12-12
**Type**: Static UI (No persistent data)

## Overview

This feature is a static landing page with no database or API requirements. This document defines the TypeScript interfaces for component props and configuration data.

## Component Interfaces

### CurriculumCard

```typescript
interface CurriculumCardProps {
  icon: React.ComponentType<{ className?: string }>;
  title: string;
  subtitle: string;
  weekRange: string;
}

// Example data
const curriculumData: CurriculumCardProps[] = [
  {
    icon: Network,
    title: "The Nervous System",
    subtitle: "ROS 2 Middleware & Nodes",
    weekRange: "Weeks 1-5"
  },
  {
    icon: Box,
    title: "The Digital Twin",
    subtitle: "Simulation in Gazebo & Unity",
    weekRange: "Weeks 6-7"
  },
  {
    icon: BrainCircuit,
    title: "The Brain",
    subtitle: "NVIDIA Isaac & Perception",
    weekRange: "Weeks 8-10"
  },
  {
    icon: Bot,
    title: "The Body",
    subtitle: "VLA & Humanoid Control",
    weekRange: "Weeks 11-13"
  }
];
```

### HardwareItem

```typescript
interface HardwareItemProps {
  category: string;      // "Brain", "Eyes", "Body"
  name: string;          // "NVIDIA Jetson Orin"
  description?: string;
  isOptional?: boolean;
}

// Example data
const hardwareData: HardwareItemProps[] = [
  {
    category: "Brain",
    name: "NVIDIA Jetson Orin",
    description: "Edge AI computing platform"
  },
  {
    category: "Eyes",
    name: "Intel RealSense",
    description: "Depth-sensing camera"
  },
  {
    category: "Body",
    name: "Unitree Go2 or G1",
    description: "Quadruped or humanoid robot",
    isOptional: true
  }
];
```

### FeatureDemo

```typescript
interface FeatureDemoProps {
  title: string;
  description: string;
  visual: React.ReactNode;
  reversed?: boolean;    // For zig-zag layout
}

// Configuration
const featuresData: FeatureDemoProps[] = [
  {
    title: "Never Get Stuck",
    description: "Ask the RAG Chatbot",
    visual: <ChatbotDemo />,
    reversed: false
  },
  {
    title: "Hardware or Software?",
    description: "You Choose",
    visual: <PersonalizationDemo />,
    reversed: true
  }
];
```

### Button Variants

```typescript
type ButtonVariant = 'primary' | 'secondary';

interface GlowButtonProps {
  variant: ButtonVariant;
  href: string;
  children: React.ReactNode;
  className?: string;
}

// Variant styles
const buttonStyles: Record<ButtonVariant, string> = {
  primary: `
    bg-cyan-400 text-slate-950 font-semibold
    shadow-[0_0_20px_rgba(34,211,238,0.5)]
    hover:shadow-[0_0_30px_rgba(34,211,238,0.7)]
  `,
  secondary: `
    bg-transparent border-2 border-white text-white
    hover:bg-white/10
  `
};
```

### ChatMessage (for ChatbotDemo)

```typescript
interface ChatMessage {
  role: 'user' | 'assistant';
  content: string;
}

// Demo conversation
const demoConversation: ChatMessage[] = [
  {
    role: 'user',
    content: 'What are quaternions and why are they used in robotics?'
  },
  {
    role: 'assistant',
    content: 'Quaternions are a mathematical representation for 3D rotations. They avoid gimbal lock and are more computationally efficient than rotation matrices...'
  }
];
```

### AnimatedSection

```typescript
interface AnimatedSectionProps {
  children: React.ReactNode;
  className?: string;
  delay?: number;        // Animation delay in seconds
}
```

## Color Palette Constants

```typescript
// Design system colors (from spec)
const colors = {
  deepVoid: '#020617',       // bg-slate-950
  textPrimary: '#f8fafc',    // text-slate-50
  textMuted: '#94a3b8',      // text-slate-400
  electricCyan: '#22d3ee',   // cyan-400
  deepViolet: '#7c3aed',     // violet-600
  glassBg: 'rgba(15, 23, 42, 0.5)',  // slate-900/50
  glassBorder: 'rgba(255, 255, 255, 0.1)',
} as const;
```

## Navigation Links

```typescript
interface NavLink {
  label: string;
  href: string;
}

const ctaLinks: Record<string, NavLink> = {
  startReading: {
    label: 'Start Reading',
    href: '/docs/module-01/overview'
  },
  watchDemo: {
    label: 'Watch Demo',
    href: '#demo'  // Or external video URL
  },
  createProfile: {
    label: 'Create Free Profile',
    href: '/signup'
  }
};
```

## State Management

No global state required. All components are stateless presentation components.

Local component state:
- `PersonalizationDemo`: `useState<'code' | 'hardware'>` for toggle state
- `ChatbotDemo`: Animation timing state (optional)

## Validation Rules

No user input fields on landing page. Validation N/A.

## Entity Relationships

```
Landing Page
├── HeroSection
│   └── GlowButton (primary, secondary)
├── CurriculumSection
│   └── CurriculumCard[] (4 items)
├── FeaturesSection
│   ├── FeatureDemo (AI Tutor)
│   │   └── ChatbotDemo
│   └── FeatureDemo (Personalization)
│       └── PersonalizationDemo
├── LabSection
│   └── HardwareItem[] (3 items)
└── FooterCTA
    └── GlowButton (primary)
```
