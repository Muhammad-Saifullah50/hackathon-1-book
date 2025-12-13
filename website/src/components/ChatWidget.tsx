import React, { useState } from 'react';
import { ChatKit, useChatKit } from '@openai/chatkit-react';
import { X, MessageCircle, Bot } from 'lucide-react';
import { useSafeColorMode } from '../hooks/useSafeColorMode';

export function ChatWidget() {
  const { isDark } = useSafeColorMode();
  const [isOpen, setIsOpen] = useState(false);

  const { control, sendUserMessage } = useChatKit({
    api: {
      url: 'http://localhost:8000/chatkit', // Backend endpoint
      domainKey: 'mock-domain-key', // Required by CustomApiConfig
    },
    theme: {
      colorScheme: isDark ? 'dark' : 'light',
      color: {
        accent: {
          primary: isDark ? "#34d399" : "#059669", // Emerald-400 (dark) / Emerald-600 (light)
          level: 2
        },
      },
      radius: "round",
      density: "normal",
      typography: { fontFamily: "system-ui, sans-serif" },
    },
    header: {
      enabled: false, // We use our own custom header
    },
    startScreen: {
      greeting: "Hi! I'm your Robotics Tutor.",
      prompts: [
        {
          label: "Explain ROS 2",
          prompt: "Can you explain the core concepts of ROS 2?",
          icon: 'agent'
        },
        {
          label: "Isaac Sim Setup",
          prompt: "How do I set up Isaac Sim for the first time?",
          icon: "bolt"
        },
        {
          label: "VLA Architecture",
          prompt: "What is the Vision-Language-Action architecture?",
          icon: "search"
        },
      ],
    },
    composer: {
      placeholder: "Ask about ROS 2, Isaac Sim, or VLA...",
    },
  });

  // Expose global handler for SelectionPopup
  React.useEffect(() => {
    (window as any).openRagChat = (text: string) => {
      setIsOpen(true); // Open the chat when triggered externally
      sendUserMessage(text);
    };
  }, [sendUserMessage]);

  // Closed state - show floating button
  if (!isOpen) {
    return (
      <button
        onClick={() => setIsOpen(true)}
        className={`fixed bottom-4 right-4 z-50 w-14 h-14 rounded-full shadow-2xl flex items-center justify-center transition-all hover:scale-105 ${
          isDark
            ? 'bg-emerald-500 hover:bg-emerald-400 shadow-emerald-500/25'
            : 'bg-emerald-600 hover:bg-emerald-500 shadow-emerald-600/25'
        }`}
        aria-label="Open chat"
      >
        <MessageCircle className="w-6 h-6 text-white" />
      </button>
    );
  }

  // Open state - show full chat widget
  return (
    <div
      className={`fixed bottom-4 right-4 z-50 w-[350px] h-[500px] shadow-2xl rounded-xl overflow-hidden font-sans flex flex-col ${
        isDark
          ? 'border-2 border-emerald-500/30 bg-slate-950'
          : 'border-2 border-slate-300 bg-white'
      }`}
    >
      {/* Custom Header */}
      <div
        className={`flex items-center justify-between px-3 py-1.5 ${
          isDark
            ? 'border-b border-emerald-500/20'
            : 'border-b border-slate-200'
        }`}
      >
        <Bot className={`w-5 h-5 ${isDark ? 'text-emerald-400' : 'text-emerald-600'}`} />
        <button
          onClick={() => setIsOpen(false)}
          className={`p-0 bg-transparent border-0 outline-none cursor-pointer transition-colors ${
            isDark
              ? 'text-slate-400 hover:text-slate-200'
              : 'text-slate-500 hover:text-slate-700'
          }`}
          aria-label="Close chat"
        >
          <X className="w-5 h-5" />
        </button>
      </div>

      {/* ChatKit Content */}
      <div className="flex-1 relative overflow-hidden">
        <ChatKit control={control} className="h-full w-full" />
      </div>
    </div>
  );
}