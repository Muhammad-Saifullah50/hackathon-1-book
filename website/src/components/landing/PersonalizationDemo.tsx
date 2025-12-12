import { useState } from 'react';
import { motion, AnimatePresence, useReducedMotion } from 'framer-motion';
import { Code, Cpu, ToggleLeft, ToggleRight } from 'lucide-react';
import { useSafeColorMode } from '../../hooks/useSafeColorMode';

/**
 * Interactive toggle demo showing code/hardware personalization.
 * Demonstrates the personalization feature of the learning platform.
 * Supports light/dark mode theming.
 */
export function PersonalizationDemo() {
  const [mode, setMode] = useState<'code' | 'hardware'>('code');
  const shouldReduceMotion = useReducedMotion();
  const { isDark } = useSafeColorMode();

  const toggleMode = () => {
    setMode((prev) => (prev === 'code' ? 'hardware' : 'code'));
  };

  return (
    <div className="w-full max-w-md mx-auto">
      {/* Demo frame */}
      <div
        className={`backdrop-blur-md rounded-2xl overflow-hidden shadow-xl ${
          isDark
            ? 'bg-slate-900/70 border border-white/10'
            : 'bg-white/90 border border-slate-200'
        }`}
      >
        {/* Header */}
        <div
          className={`flex items-center justify-between px-4 py-3 ${
            isDark
              ? 'border-b border-white/10 bg-slate-800/50'
              : 'border-b border-slate-200 bg-slate-50'
          }`}
        >
          <div className="flex items-center gap-3">
            <div className="flex items-center justify-center w-8 h-8 rounded-full bg-gradient-to-br from-amber-500 to-emerald-500">
              {mode === 'code' ? (
                <Code className="w-4 h-4 text-white" />
              ) : (
                <Cpu className="w-4 h-4 text-white" />
              )}
            </div>
            <div>
              <p className={`text-sm font-medium ${isDark ? 'text-slate-50' : 'text-slate-900'}`}>
                Learning Path
              </p>
              <p className={`text-xs ${isDark ? 'text-slate-400' : 'text-slate-500'}`}>
                {mode === 'code' ? 'Software Focus' : 'Hardware Focus'}
              </p>
            </div>
          </div>

          {/* Toggle button */}
          <button
            onClick={toggleMode}
            className={`flex items-center gap-2 px-3 py-1.5 rounded-full transition-colors ${
              isDark
                ? 'bg-slate-800/80 border border-white/10 hover:bg-slate-700/80'
                : 'bg-white border border-slate-200 hover:bg-slate-100'
            }`}
            aria-label={`Switch to ${mode === 'code' ? 'hardware' : 'code'} focus`}
          >
            {mode === 'code' ? (
              <ToggleLeft className={`w-5 h-5 ${isDark ? 'text-emerald-400' : 'text-emerald-600'}`} />
            ) : (
              <ToggleRight className={`w-5 h-5 ${isDark ? 'text-amber-400' : 'text-amber-600'}`} />
            )}
            <span className={`text-xs ${isDark ? 'text-slate-300' : 'text-slate-600'}`}>Switch</span>
          </button>
        </div>

        {/* Content area */}
        <div className="p-4 min-h-[200px] relative">
          <AnimatePresence mode="wait">
            {mode === 'code' ? (
              <motion.div
                key="code"
                initial={shouldReduceMotion ? false : { opacity: 0, x: -20 }}
                animate={{ opacity: 1, x: 0 }}
                exit={shouldReduceMotion ? undefined : { opacity: 0, x: 20 }}
                transition={{ duration: 0.3 }}
                className="space-y-3"
              >
                {/* Code snippet preview */}
                <div
                  className={`rounded-lg p-4 font-mono text-sm ${
                    isDark ? 'bg-slate-950/80' : 'bg-slate-100'
                  }`}
                >
                  <div className={`mb-2 ${isDark ? 'text-slate-500' : 'text-slate-500'}`}>
                    # Python ROS 2 Node
                  </div>
                  <div>
                    <span className={isDark ? 'text-amber-400' : 'text-amber-600'}>import</span>{' '}
                    <span className={isDark ? 'text-emerald-400' : 'text-emerald-600'}>rclpy</span>
                  </div>
                  <div>
                    <span className={isDark ? 'text-amber-400' : 'text-amber-600'}>from</span>{' '}
                    <span className={isDark ? 'text-emerald-400' : 'text-emerald-600'}>rclpy.node</span>{' '}
                    <span className={isDark ? 'text-amber-400' : 'text-amber-600'}>import</span>{' '}
                    <span className={isDark ? 'text-slate-200' : 'text-slate-800'}>Node</span>
                  </div>
                  <div className="mt-2">
                    <span className={isDark ? 'text-amber-400' : 'text-amber-600'}>class</span>{' '}
                    <span className={isDark ? 'text-amber-300' : 'text-amber-700'}>RobotController</span>
                    <span className={isDark ? 'text-slate-200' : 'text-slate-800'}>(Node):</span>
                  </div>
                  <div className="pl-4">
                    <span className={isDark ? 'text-amber-400' : 'text-amber-600'}>def</span>{' '}
                    <span className={isDark ? 'text-emerald-400' : 'text-emerald-600'}>__init__</span>
                    <span className={isDark ? 'text-slate-200' : 'text-slate-800'}>(self):</span>
                  </div>
                  <div className="pl-8">
                    <span className={isDark ? 'text-slate-200' : 'text-slate-800'}>super().</span>
                    <span className={isDark ? 'text-emerald-400' : 'text-emerald-600'}>__init__</span>
                    <span className={isDark ? 'text-slate-200' : 'text-slate-800'}>(</span>
                    <span className={isDark ? 'text-emerald-300' : 'text-emerald-700'}>'controller'</span>
                    <span className={isDark ? 'text-slate-200' : 'text-slate-800'}>)</span>
                  </div>
                </div>

                {/* Tags */}
                <div className="flex flex-wrap gap-2">
                  <span
                    className={`px-2 py-1 text-xs rounded-full ${
                      isDark
                        ? 'bg-emerald-500/10 text-emerald-400 border border-emerald-500/20'
                        : 'bg-emerald-100 text-emerald-700 border border-emerald-200'
                    }`}
                  >
                    Python
                  </span>
                  <span
                    className={`px-2 py-1 text-xs rounded-full ${
                      isDark
                        ? 'bg-amber-500/10 text-amber-400 border border-amber-500/20'
                        : 'bg-amber-100 text-amber-700 border border-amber-200'
                    }`}
                  >
                    ROS 2
                  </span>
                  <span
                    className={`px-2 py-1 text-xs rounded-full ${
                      isDark
                        ? 'bg-slate-600/10 text-slate-400 border border-slate-600/20'
                        : 'bg-slate-100 text-slate-600 border border-slate-200'
                    }`}
                  >
                    Simulation
                  </span>
                </div>
              </motion.div>
            ) : (
              <motion.div
                key="hardware"
                initial={shouldReduceMotion ? false : { opacity: 0, x: 20 }}
                animate={{ opacity: 1, x: 0 }}
                exit={shouldReduceMotion ? undefined : { opacity: 0, x: -20 }}
                transition={{ duration: 0.3 }}
                className="space-y-3"
              >
                {/* Circuit diagram preview */}
                <div className={`rounded-lg p-4 ${isDark ? 'bg-slate-950/80' : 'bg-slate-100'}`}>
                  <div className="flex items-center justify-center">
                    <svg
                      viewBox="0 0 200 120"
                      className="w-full max-w-xs h-auto"
                      fill="none"
                      stroke="currentColor"
                    >
                      {/* Circuit paths */}
                      <path
                        d="M20 60 H60 M140 60 H180"
                        className={isDark ? 'stroke-emerald-400' : 'stroke-emerald-600'}
                        strokeWidth="2"
                      />
                      <path
                        d="M60 60 Q100 20 140 60"
                        className={isDark ? 'stroke-amber-400' : 'stroke-amber-600'}
                        strokeWidth="2"
                        strokeDasharray="4 2"
                      />
                      <path
                        d="M60 60 Q100 100 140 60"
                        className={isDark ? 'stroke-amber-400' : 'stroke-amber-600'}
                        strokeWidth="2"
                        strokeDasharray="4 2"
                      />
                      {/* Nodes */}
                      <circle
                        cx="60"
                        cy="60"
                        r="8"
                        className={
                          isDark
                            ? 'fill-emerald-500/30 stroke-emerald-400'
                            : 'fill-emerald-100 stroke-emerald-600'
                        }
                        strokeWidth="2"
                      />
                      <circle
                        cx="140"
                        cy="60"
                        r="8"
                        className={
                          isDark
                            ? 'fill-emerald-500/30 stroke-emerald-400'
                            : 'fill-emerald-100 stroke-emerald-600'
                        }
                        strokeWidth="2"
                      />
                      <circle
                        cx="100"
                        cy="30"
                        r="6"
                        className={
                          isDark
                            ? 'fill-amber-500/30 stroke-amber-400'
                            : 'fill-amber-100 stroke-amber-600'
                        }
                        strokeWidth="2"
                      />
                      <circle
                        cx="100"
                        cy="90"
                        r="6"
                        className={
                          isDark
                            ? 'fill-amber-500/30 stroke-amber-400'
                            : 'fill-amber-100 stroke-amber-600'
                        }
                        strokeWidth="2"
                      />
                      {/* Labels */}
                      <text
                        x="20"
                        y="50"
                        className={`text-[8px] ${isDark ? 'fill-slate-400' : 'fill-slate-600'}`}
                      >
                        IN
                      </text>
                      <text
                        x="170"
                        y="50"
                        className={`text-[8px] ${isDark ? 'fill-slate-400' : 'fill-slate-600'}`}
                      >
                        OUT
                      </text>
                      <text
                        x="90"
                        y="15"
                        className={`text-[8px] ${isDark ? 'fill-slate-400' : 'fill-slate-600'}`}
                      >
                        Sensor
                      </text>
                      <text
                        x="90"
                        y="115"
                        className={`text-[8px] ${isDark ? 'fill-slate-400' : 'fill-slate-600'}`}
                      >
                        Motor
                      </text>
                    </svg>
                  </div>
                  <p className={`text-center text-xs mt-2 ${isDark ? 'text-slate-500' : 'text-slate-500'}`}>
                    ROS 2 Node Architecture
                  </p>
                </div>

                {/* Tags */}
                <div className="flex flex-wrap gap-2">
                  <span
                    className={`px-2 py-1 text-xs rounded-full ${
                      isDark
                        ? 'bg-amber-500/10 text-amber-400 border border-amber-500/20'
                        : 'bg-amber-100 text-amber-700 border border-amber-200'
                    }`}
                  >
                    Hardware
                  </span>
                  <span
                    className={`px-2 py-1 text-xs rounded-full ${
                      isDark
                        ? 'bg-emerald-500/10 text-emerald-400 border border-emerald-500/20'
                        : 'bg-emerald-100 text-emerald-700 border border-emerald-200'
                    }`}
                  >
                    Jetson
                  </span>
                  <span
                    className={`px-2 py-1 text-xs rounded-full ${
                      isDark
                        ? 'bg-slate-600/10 text-slate-400 border border-slate-600/20'
                        : 'bg-slate-100 text-slate-600 border border-slate-200'
                    }`}
                  >
                    Physical
                  </span>
                </div>
              </motion.div>
            )}
          </AnimatePresence>
        </div>

        {/* Footer */}
        <div
          className={`px-4 py-3 ${
            isDark
              ? 'border-t border-white/10 bg-slate-800/30'
              : 'border-t border-slate-200 bg-slate-50'
          }`}
        >
          <p className={`text-xs text-center ${isDark ? 'text-slate-500' : 'text-slate-500'}`}>
            Content adapts to your learning preference
          </p>
        </div>
      </div>
    </div>
  );
}

export default PersonalizationDemo;
