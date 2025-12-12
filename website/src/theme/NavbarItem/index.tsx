// website/src/theme/NavbarItem/index.tsx
import React from 'react';
import DefaultNavbarItem from '@theme-original/NavbarItem';
import type { Props } from '@theme/NavbarItem';
import AuthNavbarItems from '../../components/auth/AuthNavbarItems';

export default function NavbarItem(props: Props): React.ReactElement {
  // Handle custom 'auth' type navbar item
  if ((props as any).type === 'custom-auth') {
    return <AuthNavbarItems />;
  }

  // Render original NavbarItem for all other types
  return <DefaultNavbarItem {...props} />;
}