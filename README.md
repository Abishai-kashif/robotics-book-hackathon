# Chatbot Integration with Docusaurus for Textbook RAG

This project implements a conversational chatbot integrated with a Docusaurus textbook site using Qdrant as a vector database for Retrieval-Augmented Generation (RAG). The system allows students to ask natural language questions and receive answers sourced from textbook content with proper citations.

## Architecture

The system consists of:

- **Backend**: Python FastAPI application handling RAG processing, vector storage/retrieval, and response generation
- **Frontend**: React-based chat widget integrated into Docusaurus
- **Vector Database**: Qdrant for storing textbook content embeddings
- **Embedding Model**: SentenceTransformers for generating text embeddings
- **LLM**: OpenAI GPT (or similar) for response generation

## Prerequisites

- Python 3.11+
- Node.js 18+
- Docker (for running Qdrant locally)
- Access to an LLM API (OpenAI, Anthropic, or similar) or local model

## Setup

### Backend Setup

1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Configure environment variables by copying `.env.example` to `.env` and updating the values:
   ```bash
   # Qdrant configuration
   QDRANT_HOST=localhost
   QDRANT_PORT=6333

   # OpenAI API key (optional but recommended)
   OPENAI_API_KEY=your_api_key_here

   # Embedding model (default: all-MiniLM-L6-v2)
   EMBEDDING_MODEL=all-MiniLM-L6-v2
   ```

4. Start the backend server:
   ```bash
   uvicorn main:app --reload --port 8000
   ```

### Qdrant Vector Database

1. Run Qdrant locally using Docker:
   ```bash
   docker-compose up -d
   ```

### Frontend Setup

1. Navigate to the frontend directory:
   ```bash
   cd frontend
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Run the Docusaurus development server:
   ```bash
   npm run start
   ```

## API Endpoints

- `POST /api/v1/chat` - Submit a query and receive a RAG-enhanced response
- `POST /api/v1/embeddings` - Process and store new content embeddings
- `GET /api/v1/health` - Check the health status of the service
- `GET /api/v1/metrics` - Get performance metrics

## Usage

### Adding Textbook Content

To add textbook content to the system:

1. Process your textbook content using the content processor:
   ```bash
   python -m src.utils.content_processor --source-path /path/to/textbook/content
   ```

2. The processor will:
   - Split textbook content into manageable chunks
   - Generate embeddings for each chunk
   - Store embeddings in Qdrant with metadata

### Interacting with the Chatbot

The chatbot widget appears as a floating button on the bottom-right of Docusaurus pages. Click it to open the chat interface and ask questions about the textbook content.

The chatbot will:
- Understand the context of the current page
- Retrieve relevant textbook content
- Generate contextually appropriate responses
- Provide source citations for all responses

## Configuration

### Environment Variables

- `QDRANT_HOST`: Host for Qdrant service (default: localhost)
- `QDRANT_PORT`: Port for Qdrant service (default: 6333)
- `OPENAI_API_KEY`: API key for OpenAI service (optional)
- `EMBEDDING_MODEL`: Model to use for generating embeddings (default: all-MiniLM-L6-v2)

### Customization

- Adjust the chatbot UI by modifying `frontend/src/components/ChatbotWidget.css`
- Customize the embedding model in the environment variables
- Configure Qdrant settings for different scale requirements

## Development

### Running Tests

Backend tests:
```bash
cd backend
pytest
```

Frontend tests:
```bash
cd frontend
npm test
```

### Adding Features

The system is designed with the following user stories in mind:

1. **Textbook Content Q&A**: Students can ask natural language questions and receive answers sourced from textbook content
2. **Context-Aware Responses**: The chatbot understands the current page context to provide more relevant answers
3. **Search Enhancement**: Enhanced search functionality that understands user intent beyond keyword matching

## Performance

The system is designed to meet the following performance goals:

- Response time under 3 seconds for 95% of queries
- Support for concurrent users during peak usage
- Proper source citations for all responses
- Context-aware responses based on current page

## Troubleshooting

### Common Issues

1. **Qdrant Connection Issues**: Ensure Qdrant is running and accessible at the configured host/port
2. **Slow Response Times**: Check that the embedding model and LLM API key are properly configured
3. **No Results Found**: Verify that textbook content has been properly indexed in Qdrant

### Health Checks

Use the `/health` endpoint to verify the system is operational:
```bash
curl http://localhost:8000/api/v1/health
```

Use the `/metrics` endpoint to check performance metrics:
```bash
curl http://localhost:8000/api/v1/metrics
```

## Security

- API keys should be stored in environment variables, not in code
- Input validation and sanitization are implemented for all endpoints
- CORS is configured to restrict cross-origin requests (update in production)

## Deployment

For production deployment:

1. Use environment variables for all sensitive configuration
2. Set up a production-grade Qdrant instance
3. Configure proper authentication and authorization
4. Set up monitoring and logging
5. Use a reverse proxy (nginx) in front of the backend