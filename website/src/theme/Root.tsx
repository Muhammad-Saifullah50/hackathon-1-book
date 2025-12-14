/**
 * Swizzled Docusaurus Root component.
 *
 * Wraps the entire application with AuthProvider and ProfileProvider
 * to enable authentication state persistence across page navigations.
 *
 * Authentication flow:
 * 1. Better Auth manages sessions via auth-server
 * 2. AuthProvider fetches session state on mount
 * 3. JWT tokens are stored for FastAPI backend calls
 */

import React from 'react';
import { ChatWidget } from '@site/src/components/ChatWidget';
import { SelectionPopup } from '@site/src/components/SelectionPopup';
import { AuthProvider } from '../hooks/useAuth';
import { ProfileProvider } from '../hooks/useProfile';

// Root component wraps the Docusaurus application
export default function Root({children}: {children: React.ReactNode}) {
  return (
    // Wrap the entire application children with AuthProvider and ProfileProvider
    // AuthProvider now uses Better Auth for session management
    <AuthProvider>
      <ProfileProvider>
        {children}
        <SelectionPopup />
        <ChatWidget />
      </ProfileProvider>
    </AuthProvider>
  );
}