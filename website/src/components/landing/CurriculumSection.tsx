import { Network, Box, BrainCircuit, Bot } from 'lucide-react';
import { useSafeColorMode } from '../../hooks/useSafeColorMode';
import { AnimatedSection } from './AnimatedSection';
import { CurriculumCard } from './CurriculumCard';

/**
 * Curriculum data for the four main learning modules.
 */
const curriculumModules = [
  {
    icon: Network,
    title: 'The Nervous System',
    subtitle: 'ROS 2 Middleware & Nodes',
    weekRange: 'Weeks 1-5',
  },
  {
    icon: Box,
    title: 'The Digital Twin',
    subtitle: 'Simulation in Gazebo & Unity',
    weekRange: 'Weeks 6-7',
  },
  {
    icon: BrainCircuit,
    title: 'The Brain',
    subtitle: 'NVIDIA Isaac & Perception',
    weekRange: 'Weeks 8-10',
  },
  {
    icon: Bot,
    title: 'The Body',
    subtitle: 'VLA & Humanoid Control',
    weekRange: 'Weeks 11-13',
  },
];

/**
 * Curriculum section showcasing the four main learning modules
 * in a responsive grid layout.
 * Supports light/dark mode theming.
 */
export function CurriculumSection() {
  const { isDark } = useSafeColorMode();

  return (
    <section id="curriculum" className="relative py-24 lg:py-32">
      {/* Background accent */}
      <div
        className={`absolute inset-0 pointer-events-none ${
          isDark
            ? 'bg-gradient-to-b from-transparent via-emerald-950/5 to-transparent'
            : 'bg-gradient-to-b from-transparent via-emerald-50/50 to-transparent'
        }`}
      />

      <div className="container mx-auto px-6 relative z-10">
        {/* Section Header */}
        <AnimatedSection className="text-center mb-16 lg:mb-20 space-y-4">
          <span
            className={`inline-block px-4 py-2 rounded-full text-sm font-medium tracking-wide ${
              isDark
                ? 'bg-emerald-500/10 text-emerald-400 border border-emerald-500/20'
                : 'bg-emerald-100 text-emerald-700 border border-emerald-200'
            }`}
          >
            Curriculum Overview
          </span>

          <h2
            className={`text-3xl sm:text-4xl lg:text-5xl font-bold tracking-tight ${
              isDark ? 'text-slate-50' : 'text-slate-900'
            }`}
          >
            The Journey to{' '}
            <span
              className={`bg-gradient-to-r bg-clip-text text-transparent ${
                isDark ? 'from-emerald-400 to-amber-400' : 'from-emerald-600 to-amber-500'
              }`}
            >
              Embodied Intelligence
            </span>
          </h2>

          <p className={`text-lg max-w-2xl mx-auto ${isDark ? 'text-slate-400' : 'text-slate-600'}`}>
            A structured 13-week path from foundational concepts to building
            autonomous humanoid systems.
          </p>
        </AnimatedSection>

        {/* Curriculum Grid */}
        <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-6 lg:gap-8">
          {curriculumModules.map((module, index) => (
            <AnimatedSection key={module.title} delay={index * 0.1}>
              <CurriculumCard
                icon={module.icon}
                title={module.title}
                subtitle={module.subtitle}
                weekRange={module.weekRange}
              />
            </AnimatedSection>
          ))}
        </div>

        {/* Connection line for desktop - visual design element */}
        <div
          className={`hidden lg:block absolute top-1/2 left-1/2 -translate-x-1/2 w-3/4 h-px pointer-events-none ${
            isDark
              ? 'bg-gradient-to-r from-transparent via-emerald-500/20 to-transparent'
              : 'bg-gradient-to-r from-transparent via-emerald-300/40 to-transparent'
          }`}
          style={{ marginTop: '8rem' }}
        />
      </div>
    </section>
  );
}

export default CurriculumSection;
