import { useSafeColorMode } from '../../hooks/useSafeColorMode';
import { AnimatedSection } from './AnimatedSection';
import { GlowButton } from './GlowButton';

/**
 * Hero section for the landing page featuring the main headline,
 * subheadline, robot visual, and call-to-action buttons.
 * Supports light/dark mode theming.
 */
export function HeroSection() {
  const { isDark } = useSafeColorMode();

  return (
    <section className="relative min-h-[90vh] flex items-center overflow-hidden">
      {/* Background gradient overlay */}
      <div
        className={`absolute inset-0 pointer-events-none ${
          isDark
            ? 'bg-gradient-to-b from-slate-950 via-slate-950 to-slate-900/50'
            : 'bg-gradient-to-b from-white via-white to-slate-50/50'
        }`}
      />

      {/* Subtle grid pattern */}
      <div
        className={`absolute inset-0 ${isDark ? 'opacity-[0.02]' : 'opacity-[0.04]'}`}
        style={{
          backgroundImage: isDark
            ? `linear-gradient(rgba(52, 211, 153, 0.4) 1px, transparent 1px),
               linear-gradient(90deg, rgba(52, 211, 153, 0.4) 1px, transparent 1px)`
            : `linear-gradient(rgba(5, 150, 105, 0.3) 1px, transparent 1px),
               linear-gradient(90deg, rgba(5, 150, 105, 0.3) 1px, transparent 1px)`,
          backgroundSize: '60px 60px',
        }}
      />

      <div className="container mx-auto px-6 py-12 relative z-10">
        <div className="grid lg:grid-cols-2 gap-12 lg:gap-16 items-center">
          {/* Text Content */}
          <AnimatedSection className="text-center lg:text-left space-y-8">
            <h1
              className={`text-4xl sm:text-5xl lg:text-6xl xl:text-7xl font-bold tracking-tight ${
                isDark ? 'text-slate-50' : 'text-slate-900'
              }`}
            >
              Wake Up the{' '}
              <span
                className={`bg-gradient-to-r bg-clip-text text-transparent ${
                  isDark
                    ? 'from-emerald-400 to-amber-400'
                    : 'from-emerald-600 to-amber-500'
                }`}
              >
                Metal.
              </span>
            </h1>

            <p
              className={`text-lg sm:text-xl lg:text-2xl max-w-2xl mx-auto lg:mx-0 leading-relaxed ${
                isDark ? 'text-slate-400' : 'text-slate-600'
              }`}
            >
              From Python code to Walking Robots. The comprehensive guide to
              Physical AI, ROS 2, and Humanoid Mechanics.
            </p>

            <div className="flex flex-col sm:flex-row gap-4 justify-center lg:justify-start">
              <GlowButton href="/docs/module-01/overview" variant="primary">
                Start Reading
              </GlowButton>
             
            </div>

            {/* Trust indicators */}
            <div
              className={`flex flex-wrap gap-6 justify-center lg:justify-start text-sm ${
                isDark ? 'text-slate-500' : 'text-slate-600'
              }`}
            >
              <span className="flex items-center gap-2">
                <span className={`w-2 h-2 rounded-full ${isDark ? 'bg-emerald-400' : 'bg-emerald-500'}`} />
                13 Week Curriculum
              </span>
              <span className="flex items-center gap-2">
                <span className={`w-2 h-2 rounded-full ${isDark ? 'bg-amber-400' : 'bg-amber-500'}`} />
                Hands-on Projects
              </span>
              <span className="flex items-center gap-2">
                <span className={`w-2 h-2 rounded-full ${isDark ? 'bg-emerald-400' : 'bg-emerald-500'}`} />
                Free & Open Source
              </span>
            </div>
          </AnimatedSection>

          {/* Robot Visual */}
          <AnimatedSection delay={0.2} className="flex justify-center lg:justify-end">
            <div className="relative w-full max-w-md lg:max-w-lg">
              {/* Glow effect behind robot */}
              <div
                className={`absolute inset-0 blur-3xl scale-150 ${
                  isDark
                    ? 'bg-gradient-to-tr from-emerald-500/15 via-transparent to-amber-500/15'
                    : 'bg-gradient-to-tr from-emerald-500/10 via-transparent to-amber-500/10'
                }`}
              />

              {/* Robot image */}
              <img
                src="/img/landing/robot-hero.svg"
                alt="Physical AI Robot"
                className="relative z-10 w-full h-auto drop-shadow-2xl"
              />

              {/* Floating accent elements */}
              <div
                className={`absolute -top-4 -right-4 w-8 h-8 rounded-full blur-sm animate-pulse ${
                  isDark ? 'bg-emerald-400/30' : 'bg-emerald-500/25'
                }`}
              />
              <div
                className={`absolute -bottom-8 -left-8 w-12 h-12 rounded-full blur-sm animate-pulse ${
                  isDark ? 'bg-amber-400/30' : 'bg-amber-500/25'
                }`}
                style={{ animationDelay: '0.5s' }}
              />
            </div>
          </AnimatedSection>
        </div>
      </div>

      {/* Bottom gradient fade */}
      <div
        className={`absolute bottom-0 left-0 right-0 h-32 pointer-events-none ${
          isDark
            ? 'bg-gradient-to-t from-slate-950 to-transparent'
            : 'bg-gradient-to-t from-white to-transparent'
        }`}
      />
    </section>
  );
}

export default HeroSection;
