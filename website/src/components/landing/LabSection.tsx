import { Cpu, Eye, Bot, Cloud } from 'lucide-react';
import { useSafeColorMode } from '../../hooks/useSafeColorMode';
import { AnimatedSection } from './AnimatedSection';
import { HardwareItem } from './HardwareItem';

/**
 * Hardware requirements for the Physical AI curriculum.
 */
const hardwareItems = [
  {
    icon: Cpu,
    category: 'Brain',
    name: 'NVIDIA Jetson Orin',
    description:
      'Edge AI computing platform for running neural networks and robot control at real-time speeds.',
    isOptional: false,
  },
  {
    icon: Eye,
    category: 'Eyes',
    name: 'Intel RealSense D435',
    description:
      'Depth-sensing camera for 3D perception, SLAM, and object detection in your robot.',
    isOptional: false,
  },
  {
    icon: Bot,
    category: 'Body',
    name: 'Unitree Go2 / G1',
    description:
      'Quadruped or humanoid robot platform for testing locomotion and physical interactions.',
    isOptional: true,
  },
];

/**
 * Lab section showcasing hardware requirements in a game-style loadout.
 * Includes cloud simulation alternative for users without physical hardware.
 * Supports light/dark mode theming.
 */
export function LabSection() {
  const { isDark } = useSafeColorMode();

  return (
    <section className="relative py-24 lg:py-32">
      {/* Tech grid background */}
      <div
        className={isDark ? 'absolute inset-0 opacity-[0.02]' : 'absolute inset-0 opacity-[0.04]'}
        style={{
          backgroundImage: isDark
            ? `
              linear-gradient(rgba(52, 211, 153, 0.4) 1px, transparent 1px),
              linear-gradient(90deg, rgba(52, 211, 153, 0.4) 1px, transparent 1px)
            `
            : `
              linear-gradient(rgba(5, 150, 105, 0.3) 1px, transparent 1px),
              linear-gradient(90deg, rgba(5, 150, 105, 0.3) 1px, transparent 1px)
            `,
          backgroundSize: '30px 30px',
        }}
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
            Hardware Lab
          </span>

          <h2
            className={`text-3xl sm:text-4xl lg:text-5xl font-bold tracking-tight ${
              isDark ? 'text-slate-50' : 'text-slate-900'
            }`}
          >
            Required{' '}
            <span
              className={`bg-gradient-to-r bg-clip-text text-transparent ${
                isDark ? 'from-emerald-400 to-amber-400' : 'from-emerald-600 to-amber-500'
              }`}
            >
              Equipment
            </span>
          </h2>

          <p className={`text-lg max-w-2xl mx-auto ${isDark ? 'text-slate-400' : 'text-slate-600'}`}>
            Build your robotics lab or use our cloud simulation. Either way,
            you'll gain hands-on experience.
          </p>
        </AnimatedSection>

        {/* Hardware Grid - Game Loadout Style */}
        <div className="max-w-4xl mx-auto">
          {/* Loadout frame */}
          <div
            className={`relative p-1 rounded-2xl ${
              isDark
                ? 'bg-gradient-to-r from-emerald-500/20 via-transparent to-amber-500/20'
                : 'bg-gradient-to-r from-emerald-300/30 via-transparent to-amber-300/30'
            }`}
          >
            <div
              className={`backdrop-blur-sm rounded-xl p-6 lg:p-8 ${
                isDark ? 'bg-slate-950/90' : 'bg-white/95'
              }`}
            >
              {/* Loadout header */}
              <div
                className={`flex items-center justify-between mb-8 pb-4 ${
                  isDark ? 'border-b border-white/10' : 'border-b border-slate-200'
                }`}
              >
                <div className="flex items-center gap-3">
                  <div
                    className={`w-3 h-3 rounded-full animate-pulse ${
                      isDark ? 'bg-emerald-400' : 'bg-emerald-500'
                    }`}
                  />
                  <span
                    className={`text-sm font-mono uppercase tracking-wider ${
                      isDark ? 'text-slate-400' : 'text-slate-500'
                    }`}
                  >
                    Loadout Configuration
                  </span>
                </div>
                <span className={`text-xs font-mono ${isDark ? 'text-slate-500' : 'text-slate-400'}`}>
                  v2025.1
                </span>
              </div>

              {/* Hardware items grid */}
              <div className="grid grid-cols-1 md:grid-cols-3 gap-4 lg:gap-6">
                {hardwareItems.map((item, index) => (
                  <AnimatedSection key={item.category} delay={index * 0.1}>
                    <HardwareItem
                      icon={item.icon}
                      category={item.category}
                      name={item.name}
                      description={item.description}
                      isOptional={item.isOptional}
                    />
                  </AnimatedSection>
                ))}
              </div>
            </div>
          </div>
        </div>

        {/* Cloud Alternative Banner */}
        <AnimatedSection delay={0.3} className="mt-12 max-w-2xl mx-auto">
          <div
            className={`flex items-center gap-4 p-5 rounded-xl ${
              isDark
                ? 'bg-gradient-to-r from-amber-900/20 to-emerald-900/20 border border-white/10'
                : 'bg-gradient-to-r from-amber-50 to-emerald-50 border border-slate-200'
            }`}
          >
            <div
              className={`flex-shrink-0 w-12 h-12 rounded-full flex items-center justify-center ${
                isDark ? 'bg-amber-500/15' : 'bg-amber-100'
              }`}
            >
              <Cloud className={`w-6 h-6 ${isDark ? 'text-amber-400' : 'text-amber-600'}`} />
            </div>
            <div>
              <h4
                className={`text-sm font-semibold mb-1 ${
                  isDark ? 'text-slate-50' : 'text-slate-900'
                }`}
              >
                Cloud Simulation Available
              </h4>
              <p className={`text-sm ${isDark ? 'text-slate-400' : 'text-slate-600'}`}>
                Don't have hardware? Run everything in our cloud-based Gazebo
                and Isaac Sim environments. Full curriculum access included.
              </p>
            </div>
          </div>
        </AnimatedSection>
      </div>
    </section>
  );
}

export default LabSection;
