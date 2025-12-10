import React, { useState } from 'react';
import ChatDrawer from './ChatDrawer';
import styles from './styles.module.css';

interface ChatBotProps {
  selectedText?: string;
  chapterSlug?: string;
}

export default function ChatBot({ selectedText, chapterSlug }: ChatBotProps): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);

  return (
    <>
      {/* Floating chat button */}
      <button
        className={styles.floatingButton}
        onClick={() => setIsOpen(true)}
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

      {/* Chat drawer */}
      <ChatDrawer
        isOpen={isOpen}
        onClose={() => setIsOpen(false)}
        selectedText={selectedText}
        chapterSlug={chapterSlug}
      />
    </>
  );
}
