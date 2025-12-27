/**
 * Chatbot widget component for Docusaurus integration
 */
import React, { useState, useRef, useEffect } from 'react';
import ApiClient from '../services/api_client';
import './ChatbotWidget.css';

const ChatbotWidget = ({ initialContext = null }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [currentSourcePage, setCurrentSourcePage] = useState(initialContext || null);
  const [sessionId, setSessionId] = useState(null);

  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Update source page when component mounts or context changes
  useEffect(() => {
    if (typeof window !== 'undefined') {
      setCurrentSourcePage(window.location.pathname);
    }
  }, []);

  // Scroll to bottom of messages when they change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date(),
    };

    // Add user message to the chat
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Send query to the backend
      const response = await ApiClient.sendQuery(
        inputValue,
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
    } catch (error) {
      console.error('Error sending message:', error);

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
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && inputRef.current) {
      setTimeout(() => inputRef.current?.focus(), 100);
    }
  };

  const startNewChat = () => {
    setMessages([]);
    setSessionId(null);
  };

  return (
    <div className="chatbot-widget">
      {isOpen ? (
        <div className="chat-container">
          <div className="chat-header">
            <div className="chat-header-content">
              <h3>Textbook Assistant</h3>
              <div className="chat-header-actions">
                <button
                  onClick={startNewChat}
                  className="new-chat-btn"
                  title="Start new conversation"
                >
                  ✨
                </button>
                <button
                  onClick={toggleChat}
                  className="close-btn"
                  title="Close chat"
                >
                  ×
                </button>
              </div>
            </div>
          </div>

          <div className="chat-messages">
            {messages.length === 0 ? (
              <div className="welcome-message">
                <h4>Hello! I'm your textbook assistant.</h4>
                <p>Ask me anything about the robotics content, and I'll help you find relevant information from the textbook.</p>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`message ${message.sender}-message`}
                >
                  <div className="message-content">
                    <div className="message-text">{message.text}</div>

                    {message.sources && message.sources.length > 0 && (
                      <div className="message-sources">
                        <strong>Sources:</strong>
                        <ul>
                          {message.sources.map((source, idx) => (
                            <li key={idx}>
                              <a
                                href={source.source_path}
                                target="_blank"
                                rel="noopener noreferrer"
                              >
                                {source.page_reference || source.source_path}
                              </a>
                            </li>
                          ))}
                        </ul>
                      </div>
                    )}

                    {message.error && (
                      <div className="error-message">
                        An error occurred. Please try rephrasing your question.
                      </div>
                    )}
                  </div>
                </div>
              ))
            )}
            {isLoading && (
              <div className="message bot-message">
                <div className="message-content">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="chat-input-area">
            <textarea
              ref={inputRef}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question about the textbook content..."
              disabled={isLoading}
              rows="1"
              className="chat-input"
            />
            <button
              onClick={handleSendMessage}
              disabled={!inputValue.trim() || isLoading}
              className="send-button"
            >
              {isLoading ? 'Sending...' : 'Send'}
            </button>
          </div>
        </div>
      ) : (
        <button className="chat-toggle-button" onClick={toggleChat}>
          <span>🤖</span> Ask Textbook
        </button>
      )}
    </div>
  );
};

export default ChatbotWidget;