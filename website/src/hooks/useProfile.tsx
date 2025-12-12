// website/src/hooks/useProfile.tsx
import { useState, useEffect, useContext, createContext } from 'react';
import { UserProfile, UserProfileUpdate } from '../data/profile-schema';
import { useAuth } from './useAuth';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext'; // Import Docusaurus context

interface ProfileContextType {
  profile: UserProfile | null;
  loadingProfile: boolean;
  profileError: string | null; // Added error state
  createOrUpdateProfile: (profileData: UserProfileUpdate) => Promise<string | null>; // Returns error message
  fetchProfile: () => Promise<void>;
}

const ProfileContext = createContext<ProfileContextType | undefined>(undefined);

export const ProfileProvider: React.FC<React.PropsWithChildren<{}>> = ({ children }) => {
  const { user, token, loading: loadingAuth } = useAuth();
  const { siteConfig } = useDocusaurusContext(); // Get siteConfig from Docusaurus context
  const API_BASE_URL = (siteConfig.customFields?.backendUrl as string) || 'http://localhost:8000'; // Get backendUrl from customFields

  const [profile, setProfile] = useState<UserProfile | null>(null);
  const [loadingProfile, setLoadingProfile] = useState<boolean>(true);
  const [profileError, setProfileError] = useState<string | null>(null); // Initialize error state

  const fetchProfile = async () => {
    if (!user || !token) {
      setProfile(null);
      setLoadingProfile(false);
      setProfileError(null);
      return;
    }

    setLoadingProfile(true);
    setProfileError(null); // Clear previous errors
    try {
      const response = await fetch(`${API_BASE_URL}/profile`, {
        headers: { 
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`
        },
      });
      if (response.ok) {
        const profileData: UserProfile = await response.json();
        setProfile(profileData);
      } else if (response.status === 404) {
        setProfile(null); // No profile found
      } else {
        const errorData = await response.json();
        setProfileError(errorData.detail || 'Failed to fetch profile.');
      }
    } catch (error) {
      setProfileError('Network error or server unreachable during profile fetch.');
    } finally {
      setLoadingProfile(false);
    }
  };

  const createOrUpdateProfile = async (profileData: UserProfileUpdate): Promise<string | null> => {
    if (!user || !token) {
      const errorMessage = 'Cannot create/update profile: User not authenticated.';
      setProfileError(errorMessage);
      return errorMessage;
    }

    setLoadingProfile(true);
    setProfileError(null); // Clear previous errors
    try {
      // For create/update, we need to ensure the user_id is set
      const dataToSend = { ...profileData, user_id: user.id };
      
      const method = profile ? 'PUT' : 'POST';

      const response = await fetch(`${API_BASE_URL}/profile`, {
        method: method,
        headers: { 
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`
        },
        body: JSON.stringify(dataToSend),
      });

      if (response.ok) {
        const updatedProfile: UserProfile = await response.json();
        setProfile(updatedProfile);
        return null; // No error
      } else {
        const errorData = await response.json();
        const errorMessage = errorData.detail || 'Failed to create or update profile.';
        setProfileError(errorMessage);
        return errorMessage;
      }
    } catch (error) {
      console.error('Fetch error:', error); // Keep error log
      const errorMessage = 'Network error or server unreachable during profile creation/update.';
      setProfileError(errorMessage);
      return errorMessage;
    } finally {
      setLoadingProfile(false);
    }
  };

  useEffect(() => {
    if (!loadingAuth) {
      fetchProfile();
    }
  }, [user, token, loadingAuth]);

  return (
    <ProfileContext.Provider value={{ profile, loadingProfile, profileError, createOrUpdateProfile, fetchProfile }}>
      {children}
    </ProfileContext.Provider>
  );
};

export const useProfile = () => {
  const context = useContext(ProfileContext);
  if (context === undefined) {
    throw new Error('useProfile must be used within a ProfileProvider');
  }
  return context;
};
