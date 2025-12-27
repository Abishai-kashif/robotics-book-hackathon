/**
 * Entry point for the chatbot widget as a Docusaurus plugin
 */
import ChatbotWidget from './components/ChatbotWidget';

export { ChatbotWidget };
export { default as useChatbot } from './hooks/useChatbot';
export { default as apiClient } from './services/api_client';

// For backward compatibility and easy import
export default ChatbotWidget;