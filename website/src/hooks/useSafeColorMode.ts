import { useState, useEffect, useCallback } from 'react';

/**
 * Hook to safely get and set color mode, with fallback for SSR and usage outside ColorModeProvider.
 * This hook reads the data-theme attribute from the document element which is set by Docusaurus.
 * Use this hook in components that may render outside the ColorModeProvider (e.g., Root.tsx children).
 *
 * For components rendered inside Layout, prefer using `useColorMode` from '@docusaurus/theme-common'.
 */
export function useSafeColorMode() {
  const [colorMode, setColorModeState] = useState<'dark' | 'light'>('dark');

  useEffect(() => {
    // Check the document attribute set by Docusaurus
    const checkColorMode = () => {
      const theme = document.documentElement.getAttribute('data-theme');
      setColorModeState(theme === 'dark' ? 'dark' : 'light');
    };

    // Initial check
    checkColorMode();

    // Watch for changes
    const observer = new MutationObserver(checkColorMode);
    observer.observe(document.documentElement, {
      attributes: true,
      attributeFilter: ['data-theme'],
    });

    return () => observer.disconnect();
  }, []);

  const setColorMode = useCallback((mode: 'dark' | 'light') => {
    document.documentElement.setAttribute('data-theme', mode);
    // Also persist to localStorage for Docusaurus
    localStorage.setItem('theme', mode);
    setColorModeState(mode);
  }, []);

  return { colorMode, setColorMode, isDark: colorMode === 'dark' };
}
