import React, { useState, useEffect } from 'react';
import { Bot } from 'lucide-react';

export function SelectionPopup() {
  const [position, setPosition] = useState<{ x: number, y: number } | null>(null);
  const [selectedText, setSelectedText] = useState('');

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      
      if (!selection || selection.isCollapsed || selection.toString().trim().length === 0) {
        setPosition(null);
        return;
      }

      const text = selection.toString().trim();
      // Only show for reasonably sized selections (e.g., > 10 chars)
      if (text.length < 5) {
        setPosition(null);
        return;
      }

      const range = selection.getRangeAt(0);
      const rect = range.getBoundingClientRect();

      // Position above the selection
      setPosition({
        x: rect.left + (rect.width / 2) - 60, // center horizontally roughly
        y: rect.top + window.scrollY - 50 // position above
      });
      setSelectedText(text);
    };

    // Use mouseup to capture final selection state
    document.addEventListener('mouseup', handleSelection);
    // Also keyup for keyboard selection
    document.addEventListener('keyup', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }, []);

  const handleClick = (e: React.MouseEvent) => {
    e.stopPropagation(); // Prevent clearing selection immediately
    if ((window as any).openRagChat) {
      (window as any).openRagChat(`Explain this context: "${selectedText}"`);
      setPosition(null);
      window.getSelection()?.removeAllRanges();
    }
  };

  if (!position) return null;

  return (
    <button
      style={{
        position: 'absolute',
        left: `${position.x}px`,
        top: `${position.y}px`,
        zIndex: 1000
      }}
      className="flex items-center gap-2 bg-zinc-900 text-white px-3 py-2 rounded-full shadow-lg hover:scale-105 transition-transform animate-in fade-in zoom-in duration-200"
      onClick={handleClick}
      onMouseDown={(e) => e.preventDefault()} // Prevent button click from clearing selection
    >
      <Bot className="w-4 h-4" />
      <span className="text-xs font-medium">Ask Tutor</span>
    </button>
  );
}
