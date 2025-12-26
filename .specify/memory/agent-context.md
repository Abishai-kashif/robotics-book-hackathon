# Physical AI & Humanoid Robotics Development Guidelines

Auto-generated from all feature plans. Last updated: 2025-12-19

## Active Technologies

- Python 3.11 for backend services
- JavaScript/TypeScript for frontend integration
- Qdrant vector database for content storage and retrieval
- OpenAI API or similar LLM service for response generation
- Sentence transformers for text embeddings
- Docusaurus for textbook website framework
- FastAPI for backend API framework
- React for frontend components

## Project Structure

```text
backend/
├── src/
│   ├── models/
│   │   ├── query.py
│   │   ├── response.py
│   │   └── embedding.py
│   ├── services/
│   │   ├── rag_service.py
│   │   ├── qdrant_service.py
│   │   └── content_processor.py
│   ├── api/
│   │   └── chatbot_router.py
│   └── utils/
│       └── text_splitter.py
└── tests/

frontend/
├── src/
│   ├── components/
│   │   └── ChatbotWidget.jsx
│   ├── services/
│   │   └── api_client.js
│   └── hooks/
│       └── useChatbot.js
└── tests/
```

## Commands

- `uvicorn src.api.main:app --reload --port 8000` - Start backend API server
- `docker run -d --name qdrant -p 6333:6333 -p 6334:6334 qdrant/qdrant` - Run Qdrant locally
- `npm run start` - Start Docusaurus development server
- `python -m src.utils.content_processor --source-path /path/to/textbook/content` - Process textbook content to embeddings

## Code Style

- Python: Follow PEP 8 standards with type hints for all functions
- JavaScript: Use functional components with hooks for React
- API endpoints: Follow REST conventions with proper HTTP status codes
- Error handling: Always return appropriate error messages with status codes

## Recent Changes

- 005-chatbot-docusaurus: Integration of conversational chatbot with RAG for textbook content access
- 004-vla-module: Added Vision-Language-Action module content with 6 comprehensive chapters
- 003-nvidia-isaac-module: Added NVIDIA Isaac module content

<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->