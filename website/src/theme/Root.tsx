import React, { useEffect } from 'react';
import { ChatWidget } from '@site/src/components/ChatWidget';
import { SelectionPopup } from '@site/src/components/SelectionPopup';
import { AuthProvider } from '../hooks/useAuth';
import { ProfileProvider } from '../hooks/useProfile';

// Helper function to extract tokens from URL (both hash and query params)
function extractAuthTokens(): { accessToken: string | null; refreshToken: string | null } {
  let accessToken: string | null = null;
  let refreshToken: string | null = null;

  // Check URL hash first (Supabase default: #access_token=...)
  const hash = window.location.hash;
  if (hash && hash.includes('access_token')) {
    const params = new URLSearchParams(hash.substring(1)); // Remove the '#'
    accessToken = params.get('access_token');
    refreshToken = params.get('refresh_token');
  }

  // Also check query params (?access_token=... or /access_token=... misparse)
  if (!accessToken) {
    const searchParams = new URLSearchParams(window.location.search);
    accessToken = searchParams.get('access_token');
    refreshToken = searchParams.get('refresh_token');
  }

  // Handle edge case: /access_token=xxx (malformed URL from Supabase config)
  // The pathname might contain the token if redirect URL was misconfigured
  if (!accessToken && window.location.pathname.includes('access_token=')) {
    const pathParts = window.location.pathname.split('access_token=');
    if (pathParts[1]) {
      const tokenPart = pathParts[1].split('&')[0];
      accessToken = tokenPart;
      // Try to get refresh token from the rest
      const remainingPath = pathParts[1];
      const refreshMatch = remainingPath.match(/refresh_token=([^&]+)/);
      if (refreshMatch) {
        refreshToken = refreshMatch[1];
      }
    }
  }

  return { accessToken, refreshToken };
}

// Default implementation, that you can customize
export default function Root({children}: {children: React.ReactNode}) {
  useEffect(() => {
    const { accessToken, refreshToken } = extractAuthTokens();

    if (accessToken) {
      localStorage.setItem('access_token', accessToken);
      if (refreshToken) {
        localStorage.setItem('refresh_token', refreshToken);
      }

      // Clean the URL and redirect to signup-wizard
      // Use history.replaceState to clean URL without reload
      window.history.replaceState({}, document.title, '/signup-wizard');

      // Force reload to ensure AuthProvider picks up the token and navigates correctly
      window.location.href = '/signup-wizard';
    }
  }, []);

  return (
    // Wrap the entire application children with AuthProvider and ProfileProvider here
    <AuthProvider>
      <ProfileProvider>
        {children}
        <SelectionPopup />
        <ChatWidget />
      </ProfileProvider>
    </AuthProvider>
  );
}