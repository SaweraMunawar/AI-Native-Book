import React, { useState, useCallback, JSX } from 'react';
import { ChatDrawer } from '@site/src/components/ChatBot';
import { SelectToAsk } from '@site/src/components/SelectToAsk';
import styles from './Root.module.css';

interface RootProps {
  children: React.ReactNode;
}

/**
 * Root component wrapper to add global components like ChatBot and SelectToAsk
 */
export default function Root({ children }: RootProps): JSX.Element {
  const [isChatOpen, setIsChatOpen] = useState(false);
  const [selectedText, setSelectedText] = useState<string | undefined>();
  const [chapterSlug, setChapterSlug] = useState<string | undefined>();

  // Handle Ask AI from text selection
  const handleAskAI = useCallback((text: string, chapter?: string) => {
    setSelectedText(text);
    setChapterSlug(chapter);
    setIsChatOpen(true);
  }, []);

  // Handle chat close
  const handleCloseChat = useCallback(() => {
    setIsChatOpen(false);
    setSelectedText(undefined);
    setChapterSlug(undefined);
  }, []);

  // Handle floating button click
  const handleOpenChat = useCallback(() => {
    setSelectedText(undefined);
    setChapterSlug(undefined);
    setIsChatOpen(true);
  }, []);

  return (
    <>
      {children}

      {/* Select-to-Ask feature */}
      <SelectToAsk onAskAI={handleAskAI} />

      {/* Floating chat button */}
      {!isChatOpen && (
        <button
          className={styles.floatingButton}
          onClick={handleOpenChat}
          aria-label="Open chat"
        >
          <svg
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
          </svg>
        </button>
      )}

      {/* Chat drawer */}
      <ChatDrawer
        isOpen={isChatOpen}
        onClose={handleCloseChat}
        selectedText={selectedText}
        chapterSlug={chapterSlug}
      />
    </>
  );
}
