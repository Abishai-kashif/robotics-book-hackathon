# Backend - Physical AI & Humanoid Robotics Textbook

FastAPI backend for the Physical AI & Humanoid Robotics textbook chatbot, featuring Qdrant Cloud vector database and OpenAI Agents SDK integration.

## Features

- **RAG-powered Chatbot**: Question-answering using textbook content embeddings
- **Qdrant Cloud**: Managed vector database for fast similarity search
- **Multi-Model Support**: Gemini (via custom OpenAI client) or OpenAI models
- **OpenAI Agents SDK**: Conversational AI agent orchestration

## Quick Start

### Prerequisites

- Python 3.10+
- Qdrant Cloud account
- Gemini API key OR OpenAI API key

### Installation

```bash
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # Linux/Mac
# or
.\venv\Scripts\activate   # Windows

# Install dependencies
pip install -r requirements.txt
```

### Environment Configuration

Copy the example environment file and configure your credentials:

```bash
cp .env.example .env
```

Edit `.env` with your credentials:

```env
# Qdrant Cloud (REQUIRED)
QDRANT_CLUSTER_ENDPOINT=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_TIMEOUT=10.0

# LLM Provider - Choose ONE:

# Option 1: Gemini (Recommended)
GEMINI_BASE_URL=https://generativelanguage.googleapis.com/v1beta
GEMINI_API_KEY=your-gemini-api-key
MODEL_NAME=gemini-2.5-flash-preview

# Option 2: OpenAI
# OPENAI_API_KEY=sk-your-openai-key
# MODEL_NAME=gpt-4-turbo

# Embeddings (Optional)
EMBEDDING_MODEL=all-MiniLM-L6-v2
```

### Verify Configuration

```bash
python scripts/validate_env.py
```

For Qdrant indexing test:
```bash
python scripts/validate_env.py --indexing
```

### Index Textbook Content

```bash
# Index all markdown files from book-source/docs
python scripts/index_content.py --source ../book-source/docs

# With custom settings
python scripts/index_content.py --source ../book-source/docs --batch-size 64 --chunk-size 1500
```

### Run the Server

```bash
cd backend
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

The API will be available at `http://localhost:8000`

## API Endpoints

### Chat Endpoint

```bash
POST /api/v1/chat
Content-Type: application/json

{
  "content": "What is ROS 2?"
}
```

Response:
```json
{
  "response_id": "uuid",
  "answer": "ROS 2 (Robot Operating System 2) is...",
  "sources": [...],
  "processing_time_ms": 1523
}
```

### Health Check

```bash
GET /api/v1/chat/health
```

Response includes:
- Qdrant Cloud connectivity status
- LLM provider configuration
- Collection status

### Metrics

```bash
GET /api/v1/chat/metrics
```

## Project Structure

```
backend/
├── src/
│   ├── config/
│   │   └── env.py              # Environment variable validation
│   ├── database/
│   │   └── qdrant_client.py    # Qdrant Cloud client factory
│   ├── agents/
│   │   ├── chatbot.py          # Textbook agent implementation
│   │   └── models.py           # Model factory (OpenAI/Gemini)
│   ├── services/
│   │   ├── qdrant_service.py   # Qdrant operations
│   │   └── rag_service.py      # RAG pipeline
│   ├── api/
│   │   └── chatbot_router.py   # FastAPI routes
│   └── models/
│       ├── query.py            # Request models
│       ├── response.py         # Response models
│       └── embedding.py        # Embedding models
├── scripts/
│   ├── index_content.py        # Embedding indexing script
│   └── validate_env.py         # Environment validation
├── tests/
│   ├── integration/
│   └── unit/
├── requirements.txt
├── .env.example
└── main.py
```

## Configuration Reference

### Environment Variables

| Variable | Required | Description |
|----------|----------|-------------|
| `QDRANT_CLUSTER_ENDPOINT` | Yes | Qdrant Cloud cluster URL (https://...) |
| `QDRANT_API_KEY` | Yes | Qdrant Cloud API key |
| `QDRANT_TIMEOUT` | No | Connection timeout in seconds (default: 10.0) |
| `GEMINI_BASE_URL` | Yes* | Gemini API endpoint (if using Gemini) |
| `GEMINI_API_KEY` | Yes* | Gemini API key (if using Gemini) |
| `OPENAI_API_KEY` | Yes* | OpenAI API key (if using OpenAI) |
| `MODEL_NAME` | Yes | LLM model identifier |
| `EMBEDDING_MODEL` | No | Sentence-transformers model (default: all-MiniLM-L6-v2) |

*Either Gemini or OpenAI configuration is required.

### Model Names

- **Gemini**: `gemini-2.5-flash-preview`, `gemini-1.5-pro`
- **OpenAI**: `gpt-4-turbo`, `gpt-4`, `gpt-3.5-turbo`

## Qdrant Cloud Setup

1. Create account at [qdrant.cloud](https://qdrant.cloud)
2. Create new cluster (free tier available)
3. Copy cluster URL from dashboard
4. Generate API key in cluster settings
5. Add to `.env` file

### Collection Schema

The `textbook_content` collection uses:
- Vector size: 384 (all-MiniLM-L6-v2 dimensions)
- Distance metric: COSINE

## Development

### Running Tests

```bash
pytest tests/ -v
```

### Code Style

```bash
# Check formatting
black --check src/

# Lint
flake8 src/
```

## Deployment

### Render

The `render.yaml` file configures automatic deployment:

```bash
# Deploy to Render
render blueprint apply render.yaml
```

### Environment Variables on Render

Set all required environment variables in Render dashboard:
- `QDRANT_CLUSTER_ENDPOINT`
- `QDRANT_API_KEY`
- `GEMINI_BASE_URL` (or `OPENAI_API_KEY`)
- `GEMINI_API_KEY` (or `OPENAI_API_KEY`)
- `MODEL_NAME`

## Troubleshooting

### Qdrant Connection Failed

```bash
# Verify credentials
python scripts/validate_env.py --qdrant-only

# Check cluster URL format (must start with https://)
# Verify API key is correct
```

### LLM API Errors

- **401**: Check API key configuration
- **429**: Rate limit - wait and retry
- **Timeout**: Increase `QDRANT_TIMEOUT` or check network

### Empty Responses

```bash
# Verify content is indexed
python scripts/validate_env.py --indexing

# Check collection exists
curl "https://your-cluster.qdrant.io/collections" \
  -H "api-key: your-api-key"
```

## Migration from Local Qdrant

If migrating from local Docker Qdrant:

1. Export data from old cluster (if needed)
2. Update `QDRANT_CLUSTER_ENDPOINT` to cloud URL
3. Remove `docker-compose.yml` references
4. Run indexing script to populate cloud:
   ```bash
   python scripts/index_content.py --source ../book-source/docs
   ```

See `MIGRATION.md` for detailed rollback procedures.
