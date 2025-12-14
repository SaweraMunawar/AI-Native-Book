import React, { JSX } from 'react';
import type { Source } from './api';
import styles from './styles.module.css';

export interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: Source[];
  confidence?: 'high' | 'medium' | 'low';
  disclaimer?: string;
}

interface ChatMessageProps {
  message: Message;
}

export default function ChatMessage({ message }: ChatMessageProps): JSX.Element {
  const isUser = message.role === 'user';

  return (
    <div className={`${styles.message} ${isUser ? styles.userMessage : styles.assistantMessage}`}>
      <div className={styles.messageContent}>
        {message.content}
      </div>

      {/* Confidence disclaimer */}
      {message.disclaimer && (
        <div className={styles.disclaimer}>
          {message.disclaimer}
        </div>
      )}

      {/* Source citations */}
      {message.sources && message.sources.length > 0 && (
        <div className={styles.sources}>
          <div className={styles.sourcesLabel}>Sources:</div>
          {message.sources.map((source, idx) => (
            <a
              key={idx}
              href={`/docs/${source.chapter_slug}${source.section_id ? `#${source.section_id.split('#')[1]}` : ''}`}
              className={styles.sourceLink}
            >
              {source.chapter_title}
              {source.section_title && ` - ${source.section_title}`}
            </a>
          ))}
        </div>
      )}

      {/* Confidence indicator */}
      {message.confidence && !isUser && (
        <div className={`${styles.confidence} ${styles[`confidence${message.confidence.charAt(0).toUpperCase() + message.confidence.slice(1)}`]}`}>
          {message.confidence === 'high' && 'High confidence'}
          {message.confidence === 'medium' && 'Medium confidence'}
          {message.confidence === 'low' && 'Low confidence'}
        </div>
      )}
    </div>
  );
}
