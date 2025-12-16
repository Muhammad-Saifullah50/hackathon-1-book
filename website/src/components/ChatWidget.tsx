import React, { useState, useRef, useEffect } from 'react';
import { ChatKit, useChatKit } from '@openai/chatkit-react';
import { X, MessageCircle, Bot } from 'lucide-react';
import { useSafeColorMode } from '../hooks/useSafeColorMode';

export function ChatWidget() {
  const { isDark } = useSafeColorMode();
  const [isOpen, setIsOpen] = useState(false);
  const [currentThreadId, setCurrentThreadId] = useState<string | null>(
    () => localStorage.getItem('chatkit_last_thread_id')
  );
  const [pendingMessage, setPendingMessage] = useState<string | null>(null);

  const { control, sendUserMessage } = useChatKit({
    api: {
      url: 'http://localhost:8000/chatkit', // Backend endpoint
      domainKey:  '', // Required by CustomApiConfig
    },
    // Don't auto-load threads to avoid ID mismatch issues
    // Users can select threads from the history panel
    // initialThread: currentThreadId || undefined,

    // Save thread ID when it changes
    onThreadChange: ({ threadId }) => {
      console.log('🔄 Thread changed:', threadId);
      setCurrentThreadId(threadId);
      if (threadId) {
        localStorage.setItem('chatkit_last_thread_id', threadId);
      } else {
        localStorage.removeItem('chatkit_last_thread_id');
      }
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
      enabled: true, 
    },
    history: {
      enabled: true,
      showDelete: true,
      showRename: true,
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
      setPendingMessage(text); // Store the message
      setIsOpen(true); // Open the chat when triggered externally
    };
  }, []);

  // Send pending message when chat opens
  React.useEffect(() => {
    if (isOpen && pendingMessage && sendUserMessage) {
      // Wait a bit for ChatKit to be ready
      const timer = setTimeout(() => {
        sendUserMessage({ text: pendingMessage });
        setPendingMessage(null); // Clear pending message
      }, 100);

      return () => clearTimeout(timer);
    }
  }, [isOpen, pendingMessage, sendUserMessage]);

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
    <div className="fixed bottom-4 right-4 z-50">
      {/* Close Button - positioned above the chat */}
      <button
        onClick={() => setIsOpen(false)}
        className={`absolute -top-10 right-0 w-8 h-8 rounded-full flex items-center justify-center transition-colors shadow-lg focus:outline-none ${
          isDark
            ? 'bg-slate-800 hover:bg-slate-700 text-gray-300 hover:text-white'
            : 'bg-white hover:bg-gray-100 text-gray-600 hover:text-gray-900'
        }`}
        aria-label="Close chat"
      >
        <X className="w-4 h-4" />
      </button>

      {/* Chat Widget */}
      <div
        className={`w-[350px] h-[500px] shadow-2xl rounded-xl overflow-hidden font-sans flex flex-col ${
          isDark
            ? 'border-2 border-emerald-500/30 bg-slate-950'
            : 'border-2 border-slate-300 bg-white'
        }`}
      >
        {/* ChatKit Content */}
        <div className="flex-1 relative overflow-hidden">
          <ChatKit control={control} className="h-full w-full" />
        </div>
      </div>
    </div>
  );
}