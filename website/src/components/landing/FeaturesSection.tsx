import { useSafeColorMode } from '../../hooks/useSafeColorMode';
import { AnimatedSection } from './AnimatedSection';
import { FeatureDemo } from './FeatureDemo';
import { ChatbotDemo } from './ChatbotDemo';
import { PersonalizationDemo } from './PersonalizationDemo';

/**
 * Features section with zig-zag layout showcasing AI Tutor
 * and Personalization demos.
 * Supports light/dark mode theming.
 */
export function FeaturesSection() {
  const { isDark } = useSafeColorMode();

  return (
    <section className="relative py-24 lg:py-32">
      {/* Background accent */}
      <div
        className={`absolute inset-0 pointer-events-none ${
          isDark
            ? 'bg-gradient-to-b from-transparent via-amber-950/5 to-transparent'
            : 'bg-gradient-to-b from-transparent via-amber-50/50 to-transparent'
        }`}
      />

      <div className="container mx-auto px-6 relative z-10">
        {/* Section Header */}
        <AnimatedSection className="text-center mb-16 lg:mb-24 space-y-4">
          <span
            className={`inline-block px-4 py-2 rounded-full text-sm font-medium tracking-wide ${
              isDark
                ? 'bg-amber-500/10 text-amber-400 border border-amber-500/20'
                : 'bg-amber-100 text-amber-700 border border-amber-200'
            }`}
          >
            Interactive Learning
          </span>

          <h2
            className={`text-3xl sm:text-4xl lg:text-5xl font-bold tracking-tight ${
              isDark ? 'text-slate-50' : 'text-slate-900'
            }`}
          >
            Learn{' '}
            <span
              className={`bg-gradient-to-r bg-clip-text text-transparent ${
                isDark ? 'from-emerald-400 to-amber-400' : 'from-emerald-600 to-amber-500'
              }`}
            >
              Your Way
            </span>
          </h2>

          <p className={`text-lg max-w-2xl mx-auto ${isDark ? 'text-slate-400' : 'text-slate-600'}`}>
            Adaptive learning tools that meet you where you are and guide you
            where you want to go.
          </p>
        </AnimatedSection>

        {/* Feature Demos - Zig-Zag Layout */}
        <div className="space-y-24 lg:space-y-32">
          {/* AI Tutor Demo */}
          <AnimatedSection>
            <FeatureDemo
              title="Never Get Stuck"
              description="Our AI-powered tutor understands robotics, ROS 2, and Physical AI. Ask questions anytime and get detailed explanations tailored to your level."
              visual={<ChatbotDemo />}
              reversed={false}
            />
          </AnimatedSection>

          {/* Personalization Demo */}
          <AnimatedSection>
            <FeatureDemo
              title="Hardware or Software?"
              description="Toggle between code-focused and hardware-focused learning paths. Whether you're building in simulation or with real robots, we've got you covered."
              visual={<PersonalizationDemo />}
              reversed={true}
            />
          </AnimatedSection>
        </div>
      </div>
    </section>
  );
}

export default FeaturesSection;
