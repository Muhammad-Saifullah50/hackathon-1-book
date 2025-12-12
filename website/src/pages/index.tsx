import type { ReactNode } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import { useSafeColorMode } from '../hooks/useSafeColorMode';

import {
  HeroSection,
  CurriculumSection,
  FeaturesSection,
  LabSection,
  FooterCTA,
} from '../components/landing';

/**
 * Main content component that uses color mode.
 */
function HomeContent() {
  const { isDark } = useSafeColorMode();

  return (
    <main className={`min-h-screen ${isDark ? 'bg-slate-950' : 'bg-white'}`}>
      <HeroSection />
      <CurriculumSection />
      <FeaturesSection />
      <LabSection />
      <FooterCTA />
    </main>
  );
}

/**
 * Home page component for the Physical AI book website.
 * Features a modern Emerald + Gold theme with light/dark mode support.
 * Five main sections:
 * - Hero: Main headline, CTAs, and robot visual
 * - Curriculum: Four-card learning journey overview
 * - Features: AI Tutor and Personalization demos
 * - Lab: Hardware requirements loadout
 * - FooterCTA: Final conversion call-to-action
 */
export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Learn Physical AI and Robotics with practical examples and deep dives. From Python code to Walking Robots."
    >
      <HomeContent />
    </Layout>
  );
}
