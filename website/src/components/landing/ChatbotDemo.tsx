import { useState, useEffect } from 'react';
import { motion, AnimatePresence, useReducedMotion } from 'framer-motion';
import { Bot, User, Sparkles } from 'lucide-react';
import { useSafeColorMode } from '../../hooks/useSafeColorMode';

interface ChatMessage {
  role: 'user' | 'assistant';
  content: string;
}

const demoConversation: ChatMessage[] = [
  {
    role: 'user',
    content: 'What are quaternions and why are they used in robotics?',
  },
  {
    role: 'assistant',
    content:
      'Quaternions are a mathematical representation for 3D rotations. They avoid gimbal lock and are more computationally efficient than rotation matrices, making them ideal for robot arm and humanoid orientation control.',
  },
];

/**
 * Animated mock chat window demonstrating the AI tutor feature.
 * Shows a pre-defined conversation with typing animation.
 * Supports light/dark mode theming.
 */
export function ChatbotDemo() {
  const [visibleMessages, setVisibleMessages] = useState<number>(0);
  const [isTyping, setIsTyping] = useState(false);
  const shouldReduceMotion = useReducedMotion();
  const { isDark } = useSafeColorMode();

  useEffect(() => {
    if (shouldReduceMotion) {
      // Show all messages immediately if reduced motion is preferred
      setVisibleMessages(demoConversation.length);
      return;
    }

    // Animate messages appearing one by one
    if (visibleMessages < demoConversation.length) {
      setIsTyping(true);
      const timer = setTimeout(() => {
        setIsTyping(false);
        setVisibleMessages((prev) => prev + 1);
      }, visibleMessages === 0 ? 1000 : 2000);

      return () => clearTimeout(timer);
    }
  }, [visibleMessages, shouldReduceMotion]);

  return (
    <div className="w-full max-w-md mx-auto">
      {/* Chat window frame */}
      <div
        className={`backdrop-blur-md rounded-2xl overflow-hidden shadow-xl ${
          isDark
            ? 'bg-slate-900/70 border border-white/10'
            : 'bg-white/90 border border-slate-200'
        }`}
      >
        {/* Header */}
        <div
          className={`flex items-center gap-3 px-4 py-3 ${
            isDark
              ? 'border-b border-white/10 bg-slate-800/50'
              : 'border-b border-slate-200 bg-slate-50'
          }`}
        >
          <div className="flex items-center justify-center w-8 h-8 rounded-full bg-gradient-to-br from-emerald-500 to-amber-500">
            <Sparkles className="w-4 h-4 text-white" />
          </div>
          <div>
            <p className={`text-sm font-medium ${isDark ? 'text-slate-50' : 'text-slate-900'}`}>
              AI Tutor
            </p>
            <p className={`text-xs ${isDark ? 'text-slate-400' : 'text-slate-500'}`}>
              Always here to help
            </p>
          </div>
          <div className="ml-auto flex items-center gap-1">
            <span className="w-2 h-2 rounded-full bg-green-400 animate-pulse" />
            <span className={`text-xs ${isDark ? 'text-slate-400' : 'text-slate-500'}`}>Online</span>
          </div>
        </div>

        {/* Messages area */}
        <div className="p-4 space-y-4 min-h-[200px]">
          <AnimatePresence mode="wait">
            {demoConversation.slice(0, visibleMessages).map((message, index) => (
              <motion.div
                key={index}
                initial={shouldReduceMotion ? false : { opacity: 0, y: 10 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.3 }}
                className={`flex gap-3 ${
                  message.role === 'user' ? 'flex-row-reverse' : ''
                }`}
              >
                {/* Avatar */}
                <div
                  className={`flex-shrink-0 w-8 h-8 rounded-full flex items-center justify-center ${
                    message.role === 'user'
                      ? isDark ? 'bg-amber-500/30' : 'bg-amber-100'
                      : isDark ? 'bg-emerald-500/30' : 'bg-emerald-100'
                  }`}
                >
                  {message.role === 'user' ? (
                    <User className={`w-4 h-4 ${isDark ? 'text-amber-400' : 'text-amber-600'}`} />
                  ) : (
                    <Bot className={`w-4 h-4 ${isDark ? 'text-emerald-400' : 'text-emerald-600'}`} />
                  )}
                </div>

                {/* Message bubble */}
                <div
                  className={`max-w-[80%] px-4 py-2 rounded-2xl text-sm ${
                    message.role === 'user'
                      ? isDark
                        ? 'bg-amber-500/15 text-slate-200 rounded-tr-md'
                        : 'bg-amber-100 text-slate-800 rounded-tr-md'
                      : isDark
                        ? 'bg-slate-800/80 text-slate-300 rounded-tl-md'
                        : 'bg-slate-100 text-slate-700 rounded-tl-md'
                  }`}
                >
                  {message.content}
                </div>
              </motion.div>
            ))}
          </AnimatePresence>

          {/* Typing indicator */}
          {isTyping && (
            <motion.div
              initial={{ opacity: 0 }}
              animate={{ opacity: 1 }}
              className="flex gap-3"
            >
              <div
                className={`flex-shrink-0 w-8 h-8 rounded-full flex items-center justify-center ${
                  isDark ? 'bg-emerald-500/30' : 'bg-emerald-100'
                }`}
              >
                <Bot className={`w-4 h-4 ${isDark ? 'text-emerald-400' : 'text-emerald-600'}`} />
              </div>
              <div
                className={`px-4 py-3 rounded-2xl rounded-tl-md ${
                  isDark ? 'bg-slate-800/80' : 'bg-slate-100'
                }`}
              >
                <div className="flex gap-1">
                  <span
                    className={`w-2 h-2 rounded-full animate-bounce ${
                      isDark ? 'bg-slate-400' : 'bg-slate-500'
                    }`}
                  />
                  <span
                    className={`w-2 h-2 rounded-full animate-bounce ${
                      isDark ? 'bg-slate-400' : 'bg-slate-500'
                    }`}
                    style={{ animationDelay: '0.1s' }}
                  />
                  <span
                    className={`w-2 h-2 rounded-full animate-bounce ${
                      isDark ? 'bg-slate-400' : 'bg-slate-500'
                    }`}
                    style={{ animationDelay: '0.2s' }}
                  />
                </div>
              </div>
            </motion.div>
          )}
        </div>

        {/* Input area (decorative) */}
        <div
          className={`px-4 py-3 ${
            isDark
              ? 'border-t border-white/10 bg-slate-800/30'
              : 'border-t border-slate-200 bg-slate-50'
          }`}
        >
          <div
            className={`flex items-center gap-2 px-4 py-2 rounded-full ${
              isDark
                ? 'bg-slate-800/50 border border-white/5'
                : 'bg-white border border-slate-200'
            }`}
          >
            <span className={`text-sm ${isDark ? 'text-slate-500' : 'text-slate-400'}`}>
              Ask anything...
            </span>
          </div>
        </div>
      </div>
    </div>
  );
}

export default ChatbotDemo;
