/**
 * API client service for chatbot integration
 */
class ApiClient {
  constructor(baseURL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000/api/v1') {
    this.baseURL = baseURL;
  }

  /**
   * Send a query to the chatbot API
   * @param {string} query - The user's query
   * @param {string} sourcePage - The current page context (optional)
   * @param {string} sessionId - The session ID for conversation context (optional)
   * @returns {Promise<Object>} The API response
   */
  async sendQuery(query, sourcePage = null, sessionId = null) {
    try {
      const requestBody = {
        content: query,
      };

      if (sourcePage) {
        requestBody.source_page = sourcePage;
      }

      if (sessionId) {
        requestBody.session_id = sessionId;
      }

      const response = await fetch(`${this.baseURL}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        throw new Error(`API request failed: ${response.status} ${response.statusText}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error sending query to API:', error);
      throw error;
    }
  }

  /**
   * Check the health status of the API
   * @returns {Promise<Object>} The health check response
   */
  async healthCheck() {
    try {
      const response = await fetch(`${this.baseURL}/health`);

      if (!response.ok) {
        throw new Error(`Health check failed: ${response.status} ${response.statusText}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error performing health check:', error);
      throw error;
    }
  }

  /**
   * Store new content embeddings
   * @param {Object} content - The content object with title, body, source_path, etc.
   * @returns {Promise<Object>} The API response
   */
  async storeEmbeddings(content) {
    try {
      const response = await fetch(`${this.baseURL}/embeddings`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(content),
      });

      if (!response.ok) {
        throw new Error(`Storing embeddings failed: ${response.status} ${response.statusText}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error storing embeddings:', error);
      throw error;
    }
  }
}

export default new ApiClient();