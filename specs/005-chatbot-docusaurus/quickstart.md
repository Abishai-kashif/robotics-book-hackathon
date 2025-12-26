# Quickstart Guide: Chatbot Integration with Docusaurus for Textbook RAG

## Prerequisites

- Python 3.11+ installed
- Node.js 18+ installed (for Docusaurus)
- Docker (for running Qdrant locally)
- Access to an LLM API (OpenAI, Anthropic, or similar) or local model

## Setup Qdrant Vector Database

1. **Run Qdrant locally using Docker:**
   ```bash
   docker run -d --name qdrant -p 6333:6333 -p 6334:6334 \
     -v $(pwd)/qdrant_storage:/qdrant/storage \
     qdrant/qdrant
   ```

2. **Install Qdrant Python client:**
   ```bash
   pip install qdrant-client
   ```

## Backend Setup

1. **Install backend dependencies:**
   ```bash
   pip install fastapi uvicorn sentence-transformers openai python-dotenv
   ```

2. **Create backend structure:**
   ```
   backend/
   ├── src/
   │   ├── models/
   │   ├── services/
   │   ├── api/
   │   └── utils/
   ├── .env
   ├── requirements.txt
   └── main.py
   ```

3. **Configure environment variables (.env):**
   ```
   QDRANT_HOST=localhost
   QDRANT_PORT=6333
   OPENAI_API_KEY=your_api_key_here
   EMBEDDING_MODEL=all-MiniLM-L6-v2
   ```

## Content Processing Pipeline

1. **Process textbook content into embeddings:**
   ```bash
   python -m src.utils.content_processor --source-path /path/to/textbook/content
   ```

2. **The processor will:**
   - Split textbook content into manageable chunks
   - Generate embeddings for each chunk
   - Store embeddings in Qdrant with metadata

## Frontend Integration

1. **Install Docusaurus chatbot component:**
   ```bash
   npm install @docusaurus/core
   ```

2. **Add the chatbot widget to your Docusaurus site:**
   - Create a React component for the chat interface
   - Integrate with Docusaurus theme components
   - Add to all pages or specific pages as needed

## Running the Application

1. **Start the backend API:**
   ```bash
   cd backend
   uvicorn src.api.main:app --reload --port 8000
   ```

2. **Integrate with Docusaurus:**
   ```bash
   cd frontend  # or main docusaurus directory
   npm run start
   ```

## API Endpoints

- `POST /chat` - Submit a query and receive a RAG-enhanced response
- `POST /embeddings` - Process and store new content embeddings
- `GET /health` - Check the health status of the service

## Testing the Integration

1. **Verify the chatbot widget appears on textbook pages**
2. **Test with sample queries related to textbook content**
3. **Verify source citations are properly displayed**
4. **Check response times and accuracy**

## Configuration Options

- Adjust embedding model in `.env` for different quality/performance trade-offs
- Configure Qdrant settings for different scale requirements
- Customize the chatbot UI to match your Docusaurus theme