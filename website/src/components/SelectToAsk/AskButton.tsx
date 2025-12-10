import React from 'react';
import styles from './styles.module.css';

interface AskButtonProps {
  position: { x: number; y: number };
  onClick: () => void;
  isTruncated?: boolean;
}

/**
 * Floating "Ask AI" button that appears on text selection
 */
export default function AskButton({
  position,
  onClick,
  isTruncated = false,
}: AskButtonProps): JSX.Element {
  return (
    <button
      className={styles.askButton}
      style={{
        left: `${position.x}px`,
        top: `${position.y}px`,
      }}
      onClick={onClick}
      title={isTruncated ? 'Text truncated to 500 characters' : 'Ask AI about this text'}
    >
      <svg
        width="16"
        height="16"
        viewBox="0 0 24 24"
        fill="none"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
      >
        <circle cx="12" cy="12" r="10" />
        <path d="M9.09 9a3 3 0 0 1 5.83 1c0 2-3 3-3 3" />
        <line x1="12" y1="17" x2="12.01" y2="17" />
      </svg>
      Ask AI
      {isTruncated && <span className={styles.truncatedBadge}>...</span>}
    </button>
  );
}
