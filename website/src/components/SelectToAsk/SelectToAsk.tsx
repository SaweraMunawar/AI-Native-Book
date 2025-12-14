import React, { useState, useEffect, useCallback, JSX } from 'react';
import AskButton from './AskButton';
import styles from './styles.module.css';

interface SelectToAskProps {
  onAskAI: (selectedText: string, chapterSlug?: string) => void;
  maxLength?: number;
}

interface SelectionState {
  text: string;
  position: { x: number; y: number };
}

/**
 * SelectToAsk component listens for text selection and shows an "Ask AI" button
 */
export default function SelectToAsk({
  onAskAI,
  maxLength = 500,
}: SelectToAskProps): JSX.Element | null {
  const [selection, setSelection] = useState<SelectionState | null>(null);

  // Get current chapter slug from URL
  const getChapterSlug = useCallback((): string | undefined => {
    if (typeof window === 'undefined') return undefined;
    const path = window.location.pathname;
    const match = path.match(/\/docs\/([^/]+)/);
    return match ? match[1] : undefined;
  }, []);

  // Handle text selection
  const handleMouseUp = useCallback(() => {
    const windowSelection = window.getSelection();
    if (!windowSelection || windowSelection.isCollapsed) {
      setSelection(null);
      return;
    }

    const text = windowSelection.toString().trim();

    // Minimum selection threshold (10 characters)
    if (text.length < 10) {
      setSelection(null);
      return;
    }

    // Get selection position for button placement
    const range = windowSelection.getRangeAt(0);
    const rect = range.getBoundingClientRect();

    setSelection({
      text: text.length > maxLength ? text.slice(0, maxLength) : text,
      position: {
        x: rect.left + rect.width / 2,
        y: rect.top - 10,
      },
    });
  }, [maxLength]);

  // Handle click outside to dismiss
  const handleMouseDown = useCallback((e: MouseEvent) => {
    const target = e.target as HTMLElement;
    if (!target.closest(`.${styles.askButton}`)) {
      // Don't dismiss immediately, let mouseup handle new selections
    }
  }, []);

  // Handle Ask AI button click
  const handleAskClick = useCallback(() => {
    if (selection) {
      onAskAI(selection.text, getChapterSlug());
      setSelection(null);
      // Clear the selection
      window.getSelection()?.removeAllRanges();
    }
  }, [selection, onAskAI, getChapterSlug]);

  // Set up event listeners
  useEffect(() => {
    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('mousedown', handleMouseDown);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('mousedown', handleMouseDown);
    };
  }, [handleMouseUp, handleMouseDown]);

  // Don't render if no selection
  if (!selection) return null;

  const isTruncated = selection.text.length >= maxLength;

  return (
    <AskButton
      position={selection.position}
      onClick={handleAskClick}
      isTruncated={isTruncated}
    />
  );
}
