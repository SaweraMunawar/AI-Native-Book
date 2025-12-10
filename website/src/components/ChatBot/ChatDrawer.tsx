import React, { useState, useRef, useEffect } from 'react';
import ChatMessage, { type Message } from './ChatMessage';
import { sendMessage, sendContextualMessage, checkHealth, type ChatResponse } from './api';
import styles from './styles.module.css';

interface ChatDrawerProps {
  isOpen: boolean;
  onClose: () => void;
  initialMessage?: string;
  selectedText?: string;
  chapterSlug?: string;
}

export default function ChatDrawer({
  isOpen,
  onClose,
  initialMessage,
  selectedText,
  chapterSlug,
}: ChatDrawerProps): JSX.Element | null {
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState(initialMessage || '');
  const [isLoading, setIsLoading] = useState(false);
  const [isAvailable, setIsAvailable] = useState(true);
  const [sessionId, setSessionId] = useState<string | undefined>();
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Check API availability on mount
  useEffect(() => {
    checkHealth().then(setIsAvailable);
  }, []);

  // Handle initial message from text selection
  useEffect(() => {
    if (initialMessage) {
      setInput(initialMessage);
    }
  }, [initialMessage]);

  // Scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!input.trim() || isLoading) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: input.trim(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      let response: ChatResponse;

      if (selectedText) {
        response = await sendContextualMessage({
          message: userMessage.content,
          selected_text: selectedText,
          chapter_slug: chapterSlug,
          session_id: sessionId,
        });
      } else {
        response = await sendMessage({
          message: userMessage.content,
          session_id: sessionId,
        });
      }

      setSessionId(response.session_id);

      const assistantMessage: Message = {
        id: response.message_id,
        role: 'assistant',
        content: response.answer,
        sources: response.sources,
        confidence: response.confidence,
        disclaimer: response.disclaimer,
      };

      setMessages((prev) => [...prev, assistantMessage]);
    } catch (error) {
      const errorMessage: Message = {
        id: Date.now().toString(),
        role: 'assistant',
        content: `Sorry, I encountered an error: ${error instanceof Error ? error.message : 'Unknown error'}`,
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  if (!isOpen) return null;

  return (
    <div className={styles.drawer}>
      <div className={styles.drawerHeader}>
        <h3>Ask the Textbook</h3>
        <button className={styles.closeButton} onClick={onClose} aria-label="Close chat">
          ×
        </button>
      </div>

      {!isAvailable && (
        <div className={styles.unavailableNotice}>
          The chatbot is currently unavailable. Please try again later.
        </div>
      )}

      {selectedText && (
        <div className={styles.selectedTextPreview}>
          <strong>Selected text:</strong>
          <p>"{selectedText.length > 100 ? `${selectedText.slice(0, 100)}...` : selectedText}"</p>
        </div>
      )}

      <div className={styles.messagesContainer}>
        {messages.length === 0 && (
          <div className={styles.welcomeMessage}>
            Ask me anything about Physical AI, Humanoid Robotics, ROS 2, or any topic from the textbook!
          </div>
        )}
        {messages.map((msg) => (
          <ChatMessage key={msg.id} message={msg} />
        ))}
        {isLoading && (
          <div className={styles.loadingIndicator}>
            <span>Thinking...</span>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <form className={styles.inputForm} onSubmit={handleSubmit}>
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          placeholder="Ask a question..."
          disabled={isLoading || !isAvailable}
          className={styles.input}
        />
        <button
          type="submit"
          disabled={isLoading || !isAvailable || !input.trim()}
          className={styles.sendButton}
        >
          Send
        </button>
      </form>
    </div>
  );
}
