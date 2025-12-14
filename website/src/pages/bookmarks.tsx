import React, { JSX } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import { useBookmarks } from '../components/Bookmarks';
import styles from '../components/Bookmarks/styles.module.css';

function BookmarksList(): JSX.Element {
  const { bookmarks, isLoaded, removeBookmark, clearAllBookmarks } = useBookmarks();

  if (!isLoaded) {
    return (
      <div className={styles.bookmarksPage}>
        <p>Loading bookmarks...</p>
      </div>
    );
  }

  if (bookmarks.length === 0) {
    return (
      <div className={styles.bookmarksPage}>
        <div className={styles.emptyState}>
          <div className={styles.emptyIcon}>📑</div>
          <p className={styles.emptyText}>No bookmarks yet.</p>
          <p>
            Click the bookmark icon next to any section heading while reading to save it here.
          </p>
          <Link to="/docs/intro" className={styles.startReadingLink}>
            Start reading →
          </Link>
        </div>
      </div>
    );
  }

  const sortedBookmarks = [...bookmarks].sort((a, b) => b.timestamp - a.timestamp);

  return (
    <div className={styles.bookmarksPage}>
      <div className={styles.bookmarksHeader}>
        <h1 className={styles.bookmarksTitle}>Your Bookmarks</h1>
        <button
          className={styles.clearButton}
          onClick={() => {
            if (window.confirm('Are you sure you want to remove all bookmarks?')) {
              clearAllBookmarks();
            }
          }}
        >
          Clear All
        </button>
      </div>
      <ul className={styles.bookmarksList}>
        {sortedBookmarks.map((bookmark) => (
          <li key={bookmark.id} className={styles.bookmarkItem}>
            <div className={styles.bookmarkContent}>
              <Link to={bookmark.path} className={styles.bookmarkLink}>
                {bookmark.title}
              </Link>
              {bookmark.section && (
                <span className={styles.bookmarkSection}>{bookmark.section}</span>
              )}
              <span className={styles.bookmarkDate}>
                Saved {new Date(bookmark.timestamp).toLocaleDateString()}
              </span>
            </div>
            <button
              className={styles.removeButton}
              onClick={() => removeBookmark(bookmark.id)}
              title="Remove bookmark"
              aria-label="Remove bookmark"
            >
              <svg
                width="20"
                height="20"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="2"
                xmlns="http://www.w3.org/2000/svg"
              >
                <path d="M18 6L6 18M6 6l12 12" />
              </svg>
            </button>
          </li>
        ))}
      </ul>
    </div>
  );
}

export default function BookmarksPage(): JSX.Element {
  return (
    <Layout
      title="Bookmarks"
      description="Your saved bookmarks from the Physical AI & Humanoid Robotics textbook"
    >
      <main>
        <BookmarksList />
      </main>
    </Layout>
  );
}
