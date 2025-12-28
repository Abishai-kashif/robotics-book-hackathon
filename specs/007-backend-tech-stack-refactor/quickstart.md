# Quickstart Guide: Backend Tech Stack Refactor

**Feature**: 007-backend-tech-stack-refactor
**Date**: 2025-12-28
**Audience**: Developers implementing the migration to Qdrant Cloud and OpenAI Agents SDK

## Prerequisites

- Python 3.10+ installed
- Qdrant Cloud account with provisioned cluster
- API keys for LLM provider (Gemini or OpenAI)
- Git repository cloned locally

## Step 1: Provision Qdrant Cloud Cluster

1. **Create Account**: Sign up at [Qdrant Cloud](https://cloud.qdrant.io/)
2. **Create Cluster**:
   - Navigate to "Clusters" → "Create Cluster"
   - Choose free tier or paid plan
   - Select region (closest to deployment for lowest latency)
   - Wait for provisioning (2-5 minutes)
3. **Get Credentials**:
   - Cluster URL: `https://<cluster-id>.qdrant.io`
   - API Key: Generate in cluster settings (keep secure!)

## Step 2: Configure Environment Variables

Create `.env` file in `backend/` directory:

```bash
# Qdrant Cloud Configuration (REQUIRED)
QDRANT_CLUSTER_ENDPOINT=https://your-cluster-id.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here

# Gemini Configuration (Option 1 - Recommended)
GEMINI_BASE_URL=https://generativelanguage.googleapis.com/v1beta
GEMINI_API_KEY=your_gemini_api_key_here
MODEL_NAME=gemini-2.5-flash-preview

# OpenAI Configuration (Option 2 - Alternative)
# OPENAI_API_KEY=your_openai_api_key_here
# MODEL_NAME=gpt-4-turbo

# Embedding Model (Optional - defaults to all-MiniLM-L6-v2)
EMBEDDING_MODEL=all-MiniLM-L6-v2

# Application Settings
PYTHON_VERSION=3.10
```

**Security Note**: Never commit `.env` to version control. Use `.env.example` as template.

## Step 3: Install Dependencies

```bash
cd backend

# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Upgrade pip
pip install --upgrade pip

# Install dependencies
pip install -r requirements.txt
```

**New Requirements** (added to `requirements.txt`):
```
openai-agents-python>=0.2.9
openai>=1.50.0
qdrant-client>=1.7.0
sentence-transformers==2.2.2
fastapi==0.104.1
uvicorn[standard]==0.24.0
python-dotenv==1.0.0
pydantic==2.5.0
google-generativeai>=0.3.0
pytest>=7.4.0
pytest-asyncio>=0.21.0
```

## Step 4: Validate Environment Configuration

Run environment validation script:

```bash
python scripts/validate_env.py
```

Expected output:
```
✓ QDRANT_CLUSTER_ENDPOINT configured
✓ QDRANT_API_KEY configured
✓ Qdrant Cloud connection successful
✓ Gemini API key configured
✓ Gemini model accessible
✓ Embedding model loaded (all-MiniLM-L6-v2)

All checks passed! Ready to proceed.
```

If errors occur:
- **Missing env vars**: Check `.env` file matches template
- **Connection failure**: Verify Qdrant cluster URL and API key
- **Invalid API key**: Regenerate key in Qdrant/Gemini dashboards

## Step 5: Create Qdrant Cloud Collection

Run collection initialization script:

```bash
python scripts/init_qdrant_collection.py
```

This creates `textbook_content` collection with:
- **Dimensions**: 384 (matches sentence-transformers embedding size)
- **Distance**: COSINE similarity
- **Index**: HNSW for fast approximate nearest neighbor search

Expected output:
```
Creating collection: textbook_content
✓ Collection created successfully
✓ Index configured: HNSW with ef_construct=100, m=16
✓ Ready for indexing
```

## Step 6: Index Textbook Content

Run embedding generation and upload:

```bash
python scripts/index_content.py --source ../book-source/docs
```

Parameters:
- `--source`: Path to markdown files (default: `../book-source/docs`)
- `--batch-size`: Upload batch size (default: 50)
- `--chunk-size`: Text chunk size in characters (default: 1000)
- `--chunk-overlap`: Overlap between chunks (default: 200)

Expected output:
```
Reading markdown files from: ../book-source/docs
Found 47 markdown files

Processing: module-1/introduction.md
  ✓ Chunked into 5 segments
  ✓ Generated embeddings (384-dim)
  ✓ Uploaded to Qdrant Cloud

...

Summary:
  Total files: 47
  Total chunks: 312
  Total embeddings: 312
  Failed: 0
  Duration: 3m 45s
  Average: 4.8 docs/min
```

## Step 7: Test Agent Workflow

Run integration test:

```bash
pytest tests/integration/test_agent_workflow.py -v
```

Expected tests:
- `test_agent_initialization` - Verify agent creates successfully
- `test_query_with_context` - End-to-end RAG pipeline test
- `test_multi_turn_conversation` - Session management test
- `test_source_citation` - Verify citations in response

Sample test query:
```python
query = "What is ROS 2 and why is it used in humanoid robotics?"
response = await process_query(query)

assert "ROS 2" in response.answer
assert len(response.sources) > 0
assert response.confidence_score > 0.7
```

## Step 8: Start Development Server

Launch FastAPI application:

```bash
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

Expected output:
```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345]
INFO:     Started server process [12346]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

**Health Check**:
```bash
curl http://localhost:8000/api/v1/health
```

Response:
```json
{
  "status": "healthy",
  "checks": {
    "qdrant": "ok",
    "llm": "ok",
    "embeddings": "ok"
  },
  "timestamp": "2025-12-28T12:34:56Z"
}
```

## Step 9: Test API Endpoints

### Query Endpoint

```bash
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{
    "content": "Explain the difference between ROS 1 and ROS 2"
  }'
```

Response:
```json
{
  "answer": "ROS 2 differs from ROS 1 in several key ways...",
  "sources": [
    {
      "title": "Introduction to ROS 2",
      "page": "/docs/module-1/ros2-basics",
      "relevance_score": 0.89
    }
  ],
  "confidence_score": 0.89,
  "processing_time_ms": 1450.3
}
```

### Metrics Endpoint

```bash
curl http://localhost:8000/api/v1/metrics
```

Response:
```json
{
  "query_latency_p50_ms": 450.2,
  "query_latency_p95_ms": 1850.5,
  "query_latency_p99_ms": 3200.1,
  "error_rate": 0.02,
  "total_queries": 15,
  "uptime_seconds": 3600
}
```

## Step 10: Deploy to Production (Optional)

### Option A: Docker Deployment

```bash
# Build Docker image
docker build -t textbook-chatbot:latest .

# Run container with env file
docker run -d \
  --name chatbot \
  --env-file .env \
  -p 8000:8000 \
  textbook-chatbot:latest
```

### Option B: Cloud Platform (AWS/Azure/GCP)

1. **Package Application**: Create deployment artifact
2. **Configure Secrets**: Use cloud secret manager (AWS Secrets Manager, Azure Key Vault)
3. **Deploy**: Use cloud-native services (ECS, App Service, Cloud Run)
4. **Monitor**: Set up logging and alerting (CloudWatch, Application Insights)

## Troubleshooting

### Issue: Qdrant Connection Timeout

**Symptoms**: `ConnectionError: Failed to connect to Qdrant Cloud`

**Solutions**:
1. Verify cluster URL format: `https://<cluster-id>.qdrant.io`
2. Check API key is correct (regenerate if needed)
3. Ensure firewall allows outbound HTTPS (port 443)
4. Try from different network (corporate firewall may block)

### Issue: Agent Timeout (>30s)

**Symptoms**: `TimeoutError: Agent execution exceeded 30 seconds`

**Solutions**:
1. Check Gemini/OpenAI API status pages
2. Reduce context size (fewer retrieved documents)
3. Increase timeout in agent config (not recommended for production)
4. Switch to faster model (gemini-2.0-flash-exp vs gemini-1.5-pro)

### Issue: Low Confidence Scores (<0.5)

**Symptoms**: All queries return confidence < 0.5

**Solutions**:
1. Verify embeddings indexed correctly (`query_latency_p95` in metrics)
2. Check embedding model matches indexed content (384-dim)
3. Re-run indexing script with fresh embeddings
4. Review text chunking parameters (may be too large/small)

### Issue: Memory Error During Indexing

**Symptoms**: `MemoryError: Unable to allocate array`

**Solutions**:
1. Reduce batch size: `--batch-size 25`
2. Process files incrementally (subset of docs)
3. Increase system memory allocation
4. Use streaming upload (future enhancement)

## Next Steps

After completing quickstart:

1. **Run Full Test Suite**: `pytest tests/ -v --cov=backend/src`
2. **Review Metrics**: Monitor query latency and error rates
3. **Optimize Performance**: Tune chunking parameters, model selection
4. **Implement Monitoring**: Set up logging, alerting, dashboards
5. **Security Hardening**: Enable API authentication, rate limiting
6. **Documentation**: Update README with production deployment guide

## Reference Links

- [Qdrant Cloud Documentation](https://qdrant.tech/documentation/cloud/)
- [OpenAI Agents SDK GitHub](https://github.com/openai/openai-agents-python)
- [Sentence Transformers Models](https://www.sbert.net/docs/pretrained_models.html)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)

## Support

For issues or questions:
- Open GitHub issue in repository
- Check troubleshooting section above
- Review logs: `tail -f logs/application.log`
- Contact team: support@robotics-book.example.com
