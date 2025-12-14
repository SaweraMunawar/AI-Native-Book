import React from 'react';
import { useBookmarks, Bookmark } from './useBookmarks';
import styles from './styles.module.css';

interface BookmarkButtonProps {
  id: string;
  title: string;
  path: string;
  section?: string;
}

export default function BookmarkButton({
  id,
  title,
  path,
  section,
}: BookmarkButtonProps): JSX.Element {
  const { isBookmarked, toggleBookmark, isLoaded } = useBookmarks();

  if (!isLoaded) {
    return <span className={styles.bookmarkButton} />;
  }

  const bookmarked = isBookmarked(id);

  const handleClick = (e: React.MouseEvent) => {
    e.preventDefault();
    e.stopPropagation();
    toggleBookmark({ id, title, path, section });
  };

  return (
    <button
      className={`${styles.bookmarkButton} ${bookmarked ? styles.bookmarked : ''}`}
      onClick={handleClick}
      title={bookmarked ? 'Remove bookmark' : 'Add bookmark'}
      aria-label={bookmarked ? 'Remove bookmark' : 'Add bookmark'}
    >
      {bookmarked ? (
        <svg
          width="16"
          height="16"
          viewBox="0 0 24 24"
          fill="currentColor"
          xmlns="http://www.w3.org/2000/svg"
        >
          <path d="M17 3H7c-1.1 0-2 .9-2 2v16l7-3 7 3V5c0-1.1-.9-2-2-2z" />
        </svg>
      ) : (
        <svg
          width="16"
          height="16"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          xmlns="http://www.w3.org/2000/svg"
        >
          <path d="M17 3H7c-1.1 0-2 .9-2 2v16l7-3 7 3V5c0-1.1-.9-2-2-2z" />
        </svg>
      )}
    </button>
  );
}
