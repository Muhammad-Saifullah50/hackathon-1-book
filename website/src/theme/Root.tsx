import React from 'react';
import { ChatWidget } from '@site/src/components/ChatWidget';
import { SelectionPopup } from '@site/src/components/SelectionPopup';

// Default implementation, that you can customize
export default function Root({children}: {children: React.ReactNode}) {
  return (
    <>
      {children}
      <SelectionPopup />
      <ChatWidget />
    </>
  );
}
