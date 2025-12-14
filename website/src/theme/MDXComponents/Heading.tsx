import React, { JSX } from 'react';
import Heading from '@theme/Heading';
import { BookmarkButton } from '../../components/Bookmarks';
import { useLocation } from '@docusaurus/router';

type HeadingType = 'h1' | 'h2' | 'h3' | 'h4' | 'h5' | 'h6';

interface HeadingWithBookmarkProps {
  as: HeadingType;
  id?: string;
  children: React.ReactNode;
}

function HeadingWithBookmark({
  as,
  id,
  children,
  ...props
}: HeadingWithBookmarkProps): JSX.Element {
  const location = useLocation();

  // Only add bookmark button to h2 and h3 headings (sections)
  const showBookmark = (as === 'h2' || as === 'h3') && id;

  // Get text content from children
  const getTextContent = (node: React.ReactNode): string => {
    if (typeof node === 'string') return node;
    if (typeof node === 'number') return String(node);
    if (Array.isArray(node)) return node.map(getTextContent).join('');
    if (React.isValidElement(node)) {
      const { children } = node.props as { children?: React.ReactNode };
      if (children) {
        return getTextContent(children);
      }
    }
    return '';
  };

  const title = getTextContent(children);
  const path = id ? `${location.pathname}#${id}` : location.pathname;

  return (
    <Heading as={as} id={id} {...props}>
      {children}
      {showBookmark && (
        <BookmarkButton
          id={`${location.pathname}#${id}`}
          title={title}
          path={path}
          section={location.pathname.split('/').pop()?.replace(/-/g, ' ')}
        />
      )}
    </Heading>
  );
}

// Create wrapped heading components for each level
export function H1(props: Omit<HeadingWithBookmarkProps, 'as'>): JSX.Element {
  return <HeadingWithBookmark as="h1" {...props} />;
}

export function H2(props: Omit<HeadingWithBookmarkProps, 'as'>): JSX.Element {
  return <HeadingWithBookmark as="h2" {...props} />;
}

export function H3(props: Omit<HeadingWithBookmarkProps, 'as'>): JSX.Element {
  return <HeadingWithBookmark as="h3" {...props} />;
}

export function H4(props: Omit<HeadingWithBookmarkProps, 'as'>): JSX.Element {
  return <HeadingWithBookmark as="h4" {...props} />;
}

export function H5(props: Omit<HeadingWithBookmarkProps, 'as'>): JSX.Element {
  return <HeadingWithBookmark as="h5" {...props} />;
}

export function H6(props: Omit<HeadingWithBookmarkProps, 'as'>): JSX.Element {
  return <HeadingWithBookmark as="h6" {...props} />;
}

// Default export for MDXComponents compatibility
export default function MDXHeading({
  as,
  ...props
}: HeadingWithBookmarkProps): JSX.Element {
  return <HeadingWithBookmark as={as} {...props} />;
}
