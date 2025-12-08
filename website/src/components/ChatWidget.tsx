import React from 'react';
import { ChatKit, useChatKit } from '@openai/chatkit-react';
import { Book } from 'lucide-react';

export function ChatWidget() {
  const { control, sendUserMessage } = useChatKit({
    api: {
      url: 'http://localhost:8000/chatkit', // Backend endpoint
      domainKey: 'mock-domain-key', // Required by CustomApiConfig
    },
    theme: {
      colorScheme: 'dark', // Or 'auto' to match system
      color: { 
        accent: { 
          primary: "#2563EB", // Blue-600
          level: 2 
        }
      },
      radius: "round", 
      density: "normal",
      typography: { fontFamily: "system-ui, sans-serif" },
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
       sendUserMessage(text);
    };
  }, [sendUserMessage]);

  return (
    <div className="fixed bottom-4 right-4 z-50 w-[350px] h-[500px] shadow-2xl rounded-xl overflow-hidden font-sans border border-zinc-200 flex flex-col bg-white">
       <div className="flex-1 relative">
         <ChatKit control={control} className="h-full w-full" />
       </div>
    </div>
  );
}