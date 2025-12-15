import React from 'react';
import Heading from '@theme-original/Heading';
import type { Props } from '@theme/Heading';
import PersonalizationBar from '../../components/profile/PersonalizationBar';
import { useLocation } from '@docusaurus/router';

export default function HeadingWrapper(props: Props): React.ReactElement {
  const location = useLocation();
  const isDocPage = location.pathname.startsWith('/docs/');

  // Only show PersonalizationBar after h1 headings on doc pages
  const showPersonalizationBar = props.as === 'h1' && isDocPage;

  console.log('[HeadingWrapper]', {
    pathname: location.pathname,
    isDocPage,
    headingAs: props.as,
    showPersonalizationBar
  });

  return (
    <>
      <Heading {...props} />
      {showPersonalizationBar && (
        <div className="mt-4 mb-6">
          <PersonalizationBar />
        </div>
      )}
    </>
  );
}
