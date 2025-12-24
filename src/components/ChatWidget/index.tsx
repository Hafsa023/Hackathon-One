/**
 * ChatWidget - Main chat component for the Physical AI Book chatbot
 *
 * This component provides:
 * - Floating chat bubble that expands to a chat panel
 * - Text input for questions
 * - Selected text mode for context-specific questions
 * - Chat history display
 * - Responsive design for mobile and desktop
 */

import React, { useState, useRef, useEffect, useCallback } from 'react';
import styles from './styles.module.css';

// Types
interface Message {
  id: string;
  type: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  sources?: Source[];
}

interface Source {
  chapter: string;
  section?: string;
  relevance_score: number;
}

interface ChatResponse {
  answer: string;
  sources: Source[];
  session_id: string;
  timestamp: string;
}

// Configuration - Backend URL (Railway deployed)
const API_BASE_URL = 'https://brilliant-alignment-production-75b3.up.railway.app';

// Generate unique ID
const generateId = (): string => {
  return Math.random().toString(36).substring(2) + Date.now().toString(36);
};

// Custom hook for selected text detection
const useTextSelection = (onSelect: (text: string) => void) => {
  useEffect(() => {
    const handleMouseUp = () => {
      const selection = window.getSelection();
      const selectedText = selection?.toString().trim();

      if (selectedText && selectedText.length > 10) {
        onSelect(selectedText);
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    return () => document.removeEventListener('mouseup', handleMouseUp);
  }, [onSelect]);
};

// Local storage hook for session persistence
const useLocalStorage = <T,>(key: string, initialValue: T): [T, (value: T) => void] => {
  const [storedValue, setStoredValue] = useState<T>(() => {
    if (typeof window === 'undefined') return initialValue;
    try {
      const item = window.localStorage.getItem(key);
      return item ? JSON.parse(item) : initialValue;
    } catch {
      return initialValue;
    }
  });

  const setValue = (value: T) => {
    setStoredValue(value);
    if (typeof window !== 'undefined') {
      window.localStorage.setItem(key, JSON.stringify(value));
    }
  };

  return [storedValue, setValue];
};

export default function ChatWidget(): JSX.Element {
  // State
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useLocalStorage<Message[]>('chat_messages', []);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [sessionId, setSessionId] = useLocalStorage<string>('chat_session_id', generateId());
  const [error, setError] = useState<string | null>(null);

  // Refs
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  // Get current chapter from URL
  const getCurrentChapter = (): string | undefined => {
    if (typeof window === 'undefined') return undefined;
    const path = window.location.pathname;
    const match = path.match(/\/docs\/(.+)/);
    return match ? match[1].replace(/\//g, ' > ') : undefined;
  };

  // Scroll to bottom of messages
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  // Handle text selection
  const handleTextSelection = useCallback((text: string) => {
    if (text.length > 10 && text.length < 5000) {
      setSelectedText(text);
    }
  }, []);

  useTextSelection(handleTextSelection);

  // Ref to track streaming content without causing re-renders
  const streamingContentRef = useRef<string>('');
  const streamingMessageIdRef = useRef<string | null>(null);

  // Send message to backend with streaming
  const sendMessage = async (question: string, useSelectedText: boolean = false) => {
    if (!question.trim()) return;

    setError(null);
    setIsLoading(true);

    // Add user message
    const userMessage: Message = {
      id: generateId(),
      type: 'user',
      content: question,
      timestamp: new Date(),
    };
    setMessages((prev) => [...prev, userMessage]);
    setInputValue('');

    // Create placeholder for assistant message
    const assistantMessageId = generateId();
    streamingMessageIdRef.current = assistantMessageId;
    streamingContentRef.current = '';

    const assistantMessage: Message = {
      id: assistantMessageId,
      type: 'assistant',
      content: '',
      timestamp: new Date(),
    };
    setMessages((prev) => [...prev, assistantMessage]);

    try {
      // Build request body, omitting null/undefined values
      const requestBody: Record<string, string | undefined> = {
        question,
        session_id: sessionId,
      };
      if (useSelectedText && selectedText) {
        requestBody.selected_text = selectedText;
      }
      const currentChapter = getCurrentChapter();
      if (currentChapter) {
        requestBody.chapter = currentChapter;
      }

      const response = await fetch(`${API_BASE_URL}/chat/stream`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const reader = response.body?.getReader();
      const decoder = new TextDecoder();

      if (!reader) {
        throw new Error('Response body is not readable');
      }

      // Buffer for incomplete SSE data
      let buffer = '';

      while (true) {
        const { done, value } = await reader.read();
        if (done) break;

        buffer += decoder.decode(value, { stream: true });
        const lines = buffer.split('\n');

        // Keep the last incomplete line in buffer
        buffer = lines.pop() || '';

        for (const line of lines) {
          if (line.startsWith('data: ')) {
            const dataStr = line.slice(6).trim();
            if (!dataStr) continue;

            try {
              const data = JSON.parse(dataStr);

              if (data.type === 'metadata') {
                // Update session ID
                if (data.session_id) {
                  setSessionId(data.session_id);
                }
              } else if (data.type === 'content') {
                // Accumulate content in ref (no re-render)
                streamingContentRef.current += data.content;
                // Update message state
                setMessages((prev) =>
                  prev.map((msg) =>
                    msg.id === assistantMessageId
                      ? { ...msg, content: streamingContentRef.current }
                      : msg
                  )
                );
              } else if (data.type === 'done') {
                // Streaming complete
                setIsLoading(false);
              }
            } catch {
              // Ignore parse errors for incomplete chunks
            }
          }
        }
      }

      // Clear selected text after using it
      if (useSelectedText) {
        setSelectedText(null);
      }
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to get response';
      setError(errorMessage);

      // Update the assistant message to show error
      setMessages((prev) =>
        prev.map((msg) =>
          msg.id === assistantMessageId
            ? { ...msg, content: 'Sorry, I encountered an error. Please try again.' }
            : msg
        )
      );
    } finally {
      setIsLoading(false);
      streamingMessageIdRef.current = null;
    }
  };

  // Handle form submit
  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    sendMessage(inputValue, !!selectedText);
  };

  // Handle key press (Enter to send, Shift+Enter for newline)
  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e);
    }
  };

  // Clear chat history
  const clearChat = () => {
    setMessages([]);
    setSessionId(generateId());
    setSelectedText(null);
  };

  // Dismiss selected text
  const dismissSelectedText = () => {
    setSelectedText(null);
  };

  return (
    <div className={styles.chatWidgetContainer}>
      {/* Chat Toggle Button - Robot Icon */}
      {!isOpen && (
        <button
          className={styles.chatToggle}
          onClick={() => setIsOpen(true)}
          aria-label="Open chat"
          title="Ask a question about the book"
        >
          <svg
            width="28"
            height="28"
            viewBox="0 0 24 24"
            fill="none"
            xmlns="http://www.w3.org/2000/svg"
          >
            {/* Robot head */}
            <rect x="4" y="6" width="16" height="12" rx="2" stroke="currentColor" strokeWidth="2" />
            {/* Antenna */}
            <line x1="12" y1="6" x2="12" y2="3" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />
            <circle cx="12" cy="2" r="1" fill="currentColor" />
            {/* Eyes */}
            <circle cx="8.5" cy="11" r="1.5" fill="currentColor" />
            <circle cx="15.5" cy="11" r="1.5" fill="currentColor" />
            {/* Mouth */}
            <rect x="8" y="14" width="8" height="2" rx="1" fill="currentColor" opacity="0.5" />
            {/* Ear nodes */}
            <rect x="2" y="10" width="2" height="4" rx="1" fill="currentColor" />
            <rect x="20" y="10" width="2" height="4" rx="1" fill="currentColor" />
          </svg>
          {selectedText && <span className={styles.selectionIndicator} />}
        </button>
      )}

      {/* Chat Panel */}
      {isOpen && (
        <div className={styles.chatPanel}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <div className={styles.headerTitle}>
              <svg
                width="22"
                height="22"
                viewBox="0 0 24 24"
                fill="none"
                xmlns="http://www.w3.org/2000/svg"
              >
                {/* Circuit/CPU icon */}
                <rect x="6" y="6" width="12" height="12" rx="1" stroke="currentColor" strokeWidth="2" />
                <rect x="9" y="9" width="6" height="6" rx="1" fill="currentColor" opacity="0.3" />
                <line x1="12" y1="2" x2="12" y2="6" stroke="currentColor" strokeWidth="2" />
                <line x1="12" y1="18" x2="12" y2="22" stroke="currentColor" strokeWidth="2" />
                <line x1="2" y1="12" x2="6" y2="12" stroke="currentColor" strokeWidth="2" />
                <line x1="18" y1="12" x2="22" y2="12" stroke="currentColor" strokeWidth="2" />
              </svg>
              <span>ROBO-ASSIST</span>
            </div>
            <div className={styles.headerActions}>
              <button
                className={styles.headerButton}
                onClick={clearChat}
                title="Clear chat"
              >
                <svg
                  width="16"
                  height="16"
                  viewBox="0 0 24 24"
                  fill="none"
                  xmlns="http://www.w3.org/2000/svg"
                >
                  <path
                    d="M3 6H5H21M8 6V4C8 3.46957 8.21071 2.96086 8.58579 2.58579C8.96086 2.21071 9.46957 2 10 2H14C14.5304 2 15.0391 2.21071 15.4142 2.58579C15.7893 2.96086 16 3.46957 16 4V6M19 6V20C19 20.5304 18.7893 21.0391 18.4142 21.4142C18.0391 21.7893 17.5304 22 17 22H7C6.46957 22 5.96086 21.7893 5.58579 21.4142C5.21071 21.0391 5 20.5304 5 20V6H19Z"
                    stroke="currentColor"
                    strokeWidth="2"
                    strokeLinecap="round"
                    strokeLinejoin="round"
                  />
                </svg>
              </button>
              <button
                className={styles.headerButton}
                onClick={() => setIsOpen(false)}
                title="Close chat"
              >
                <svg
                  width="16"
                  height="16"
                  viewBox="0 0 24 24"
                  fill="none"
                  xmlns="http://www.w3.org/2000/svg"
                >
                  <path
                    d="M18 6L6 18M6 6L18 18"
                    stroke="currentColor"
                    strokeWidth="2"
                    strokeLinecap="round"
                    strokeLinejoin="round"
                  />
                </svg>
              </button>
            </div>
          </div>

          {/* Selected Text Banner */}
          {selectedText && (
            <div className={styles.selectedTextBanner}>
              <div className={styles.selectedTextContent}>
                <span className={styles.selectedTextLabel}>Selected text:</span>
                <span className={styles.selectedTextPreview}>
                  {selectedText.length > 100
                    ? selectedText.substring(0, 100) + '...'
                    : selectedText}
                </span>
              </div>
              <button
                className={styles.dismissButton}
                onClick={dismissSelectedText}
                title="Clear selection"
              >
                ×
              </button>
            </div>
          )}

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <h4>[ SYSTEM ONLINE ]</h4>
                <p>
                  Physical AI knowledge database loaded. Query robotics, embodied AI,
                  ROS 2, simulation protocols, and more. Select text for contextual analysis.
                </p>
                <div className={styles.suggestionList}>
                  <button
                    className={styles.suggestionButton}
                    onClick={() => sendMessage('What is embodied intelligence?')}
                  >
                    Query: Embodied Intelligence
                  </button>
                  <button
                    className={styles.suggestionButton}
                    onClick={() => sendMessage('Explain the sensorimotor loop')}
                  >
                    Query: Sensorimotor Loop
                  </button>
                  <button
                    className={styles.suggestionButton}
                    onClick={() => sendMessage('What is sim-to-real transfer?')}
                  >
                    Query: Sim-to-Real Transfer
                  </button>
                </div>
              </div>
            )}

            {messages.map((message) => (
              <div
                key={message.id}
                className={`${styles.message} ${
                  message.type === 'user' ? styles.userMessage : styles.assistantMessage
                }`}
              >
                <div className={styles.messageContent}>
                  {message.content}
                </div>
              </div>
            ))}

            {isLoading && !streamingContentRef.current && (
              <div className={`${styles.message} ${styles.assistantMessage}`}>
                <div className={styles.typingIndicator}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}

            {error && (
              <div className={styles.errorMessage}>
                {error}
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input Area */}
          <form className={styles.inputArea} onSubmit={handleSubmit}>
            <textarea
              ref={inputRef}
              className={styles.input}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder={
                selectedText
                  ? 'Ask about the selected text...'
                  : 'Ask a question about the book...'
              }
              disabled={isLoading}
              rows={1}
            />
            <button
              type="submit"
              className={styles.sendButton}
              disabled={isLoading || !inputValue.trim()}
              title="Send message"
            >
              <svg
                width="20"
                height="20"
                viewBox="0 0 24 24"
                fill="none"
                xmlns="http://www.w3.org/2000/svg"
              >
                <path
                  d="M22 2L11 13M22 2L15 22L11 13M22 2L2 9L11 13"
                  stroke="currentColor"
                  strokeWidth="2"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                />
              </svg>
            </button>
          </form>
        </div>
      )}
    </div>
  );
}
