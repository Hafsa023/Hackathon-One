import React from 'react';
import { useLocation } from '@docusaurus/router';
import ReadingProgress from '@site/src/components/ReadingProgress';
import ChatWidget from '@site/src/components/ChatWidget';

interface RootProps {
  children: React.ReactNode;
}

export default function Root({ children }: RootProps): JSX.Element {
  const location = useLocation();
  const isDocsPage = location.pathname.startsWith('/docs');

  return (
    <>
      {isDocsPage && <ReadingProgress />}
      {children}
      {isDocsPage && <ChatWidget />}
    </>
  );
}
