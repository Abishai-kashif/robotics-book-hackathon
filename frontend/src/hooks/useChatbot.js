/**
 * Custom hook for managing chatbot state and functionality
 */
import { useState, useCallback, useEffect } from 'react';
import ApiClient from '../services/api_client';

const useChatbot = (initialContext = null) => {
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const [currentSourcePage, setCurrentSourcePage] = useState(initialContext || null);
  const [error, setError] = useState(null);

  // Update source page when hook is used in a browser context
  useEffect(() => {
    if (typeof window !== 'undefined') {
      setCurrentSourcePage(window.location.pathname);
    }
  }, []);

  /**
   * Send a message to the chatbot API
   */
  const sendMessage = useCallback(async (text) => {
    if (!text.trim() || isLoading) {
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      // Add user message to the chat
      const userMessage = {
        id: Date.now(),
        text: text,
        sender: 'user',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, userMessage]);

      // Send query to the backend
      const response = await ApiClient.sendQuery(
        text,
        currentSourcePage,
        sessionId
      );

      // Create bot message with sources
      const botMessage = {
        id: Date.now() + 1,
        text: response.answer,
        sender: 'bot',
        sources: response.sources || [],
        confidence: response.confidence_score,
        timestamp: new Date(),
      };

      // Add bot message to the chat
      setMessages(prev => [...prev, botMessage]);

      // Set session ID if it was returned
      if (response.session_id && !sessionId) {
        setSessionId(response.session_id);
      }
    } catch (err) {
      console.error('Error sending message:', err);
      setError(err.message);

      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        sender: 'bot',
        error: true,
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  }, [isLoading, currentSourcePage, sessionId]);

  /**
   * Clear the current conversation
   */
  const clearConversation = useCallback(() => {
    setMessages([]);
    setSessionId(null);
    setError(null);
  }, []);

  /**
   * Update the current context (source page)
   */
  const updateContext = useCallback((newContext) => {
    setCurrentSourcePage(newContext);
  }, []);

  /**
   * Check if the API is healthy
   */
  const checkHealth = useCallback(async () => {
    try {
      const health = await ApiClient.healthCheck();
      return health.status === 'healthy';
    } catch (err) {
      console.error('Health check failed:', err);
      return false;
    }
  }, []);

  return {
    messages,
    isLoading,
    error,
    sessionId,
    sendMessage,
    clearConversation,
    updateContext,
    checkHealth,
  };
};

export default useChatbot;