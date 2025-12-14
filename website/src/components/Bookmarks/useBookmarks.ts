import { useState, useEffect, useCallback } from 'react';

export interface Bookmark {
  id: string;
  title: string;
  path: string;
  section?: string;
  timestamp: number;
}

const STORAGE_KEY = 'textbook-bookmarks';

export function useBookmarks() {
  const [bookmarks, setBookmarks] = useState<Bookmark[]>([]);
  const [isLoaded, setIsLoaded] = useState(false);

  // Load bookmarks from localStorage on mount
  useEffect(() => {
    if (typeof window !== 'undefined') {
      try {
        const stored = localStorage.getItem(STORAGE_KEY);
        if (stored) {
          setBookmarks(JSON.parse(stored));
        }
      } catch (e) {
        console.error('Failed to load bookmarks:', e);
      }
      setIsLoaded(true);
    }
  }, []);

  // Save bookmarks to localStorage whenever they change
  useEffect(() => {
    if (isLoaded && typeof window !== 'undefined') {
      try {
        localStorage.setItem(STORAGE_KEY, JSON.stringify(bookmarks));
      } catch (e) {
        console.error('Failed to save bookmarks:', e);
      }
    }
  }, [bookmarks, isLoaded]);

  const addBookmark = useCallback((bookmark: Omit<Bookmark, 'timestamp'>) => {
    setBookmarks((prev) => {
      // Check if already bookmarked
      if (prev.some((b) => b.id === bookmark.id)) {
        return prev;
      }
      return [...prev, { ...bookmark, timestamp: Date.now() }];
    });
  }, []);

  const removeBookmark = useCallback((id: string) => {
    setBookmarks((prev) => prev.filter((b) => b.id !== id));
  }, []);

  const isBookmarked = useCallback(
    (id: string) => {
      return bookmarks.some((b) => b.id === id);
    },
    [bookmarks]
  );

  const toggleBookmark = useCallback(
    (bookmark: Omit<Bookmark, 'timestamp'>) => {
      if (isBookmarked(bookmark.id)) {
        removeBookmark(bookmark.id);
      } else {
        addBookmark(bookmark);
      }
    },
    [isBookmarked, addBookmark, removeBookmark]
  );

  const clearAllBookmarks = useCallback(() => {
    setBookmarks([]);
  }, []);

  return {
    bookmarks,
    isLoaded,
    addBookmark,
    removeBookmark,
    isBookmarked,
    toggleBookmark,
    clearAllBookmarks,
  };
}
