import { useSafeColorMode } from '../../hooks/useSafeColorMode';
import { AnimatedSection } from './AnimatedSection';
import { GlowButton } from './GlowButton';

/**
 * Footer call-to-action section with gradient background,
 * final CTA, and branding badges.
 * Supports light/dark mode theming.
 */
export function FooterCTA() {
  const { isDark } = useSafeColorMode();

  return (
    <section className="relative py-24 lg:py-32">
      {/* Gradient background */}
      <div
        className={`absolute inset-0 pointer-events-none ${
          isDark
            ? 'bg-gradient-to-t from-emerald-950/20 via-slate-950 to-slate-950'
            : 'bg-gradient-to-t from-emerald-50/50 via-white to-white'
        }`}
      />

      {/* Decorative glow orbs */}
      <div
        className={`absolute bottom-0 left-1/4 w-96 h-96 rounded-full blur-3xl pointer-events-none ${
          isDark ? 'bg-emerald-500/5' : 'bg-emerald-500/8'
        }`}
      />
      <div
        className={`absolute bottom-0 right-1/4 w-64 h-64 rounded-full blur-3xl pointer-events-none ${
          isDark ? 'bg-amber-500/5' : 'bg-amber-500/8'
        }`}
      />

      <div className="container mx-auto px-6 relative z-10">
        <AnimatedSection className="text-center space-y-8">
          {/* Main headline */}
          <h2
            className={`text-4xl sm:text-5xl lg:text-6xl font-bold tracking-tight ${
              isDark ? 'text-slate-50' : 'text-slate-900'
            }`}
          >
            Build the{' '}
            <span
              className={`bg-gradient-to-r bg-clip-text text-transparent ${
                isDark ? 'from-emerald-400 to-amber-400' : 'from-emerald-600 to-amber-500'
              }`}
            >
              Future
            </span>
          </h2>

          <p
            className={`text-lg lg:text-xl max-w-xl mx-auto ${
              isDark ? 'text-slate-400' : 'text-slate-600'
            }`}
          >
            Join thousands of learners mastering Physical AI and humanoid
            robotics.
          </p>

          {/* Primary CTA */}
          <div className="pt-4">
            <GlowButton href="/signup" variant="primary">
              Create Free Profile
            </GlowButton>
          </div>

          {/* Badge section */}
          <div
            className={`pt-8 flex flex-wrap items-center justify-center gap-4 text-sm ${
              isDark ? 'text-slate-500' : 'text-slate-500'
            }`}
          >
         
          </div>
        </AnimatedSection>
      </div>
    </section>
  );
}



export default FooterCTA;
