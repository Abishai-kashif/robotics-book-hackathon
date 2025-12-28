# Research: Backend Tech Stack Refactor

**Feature**: 007-backend-tech-stack-refactor
**Date**: 2025-12-28
**Purpose**: Resolve unknowns from Technical Context and establish implementation approach

## Research Questions

### 1. Embedding Model Library Selection

**Question**: Should we use OpenAI embeddings API, sentence-transformers, or another embedding library?

**Decision**: Continue using `sentence-transformers` (currently: `all-MiniLM-L6-v2`)

**Rationale**:
- **Current Implementation**: System already uses `sentence-transformers==2.2.2` with `all-MiniLM-L6-v2` model (384-dimensional embeddings)
- **No Migration Needed**: Existing embeddings in Qdrant are compatible with current model; changing models would require full re-indexing of all textbook content
- **Cost-Effective**: sentence-transformers runs locally without API costs, critical for textbook project budget
- **Performance**: Model is optimized for semantic similarity tasks with acceptable quality for educational Q&A
- **Proven**: Successfully deployed in current chatbot implementation with good results

**Alternatives Considered**:
- **OpenAI Embeddings API** (`text-embedding-3-small`):
  - Pros: Higher quality embeddings (1536 dimensions), maintained by OpenAI
  - Cons: API costs per token, requires internet connectivity, vendor lock-in
  - **Rejected**: Cost prohibitive for educational project, adds external dependency

- **Sentence-Transformers Upgrade** (e.g., `all-mpnet-base-v2`):
  - Pros: Better quality (768 dimensions), still local
  - Cons: Requires complete re-indexing, larger model size, slower inference
  - **Rejected**: Migration cost not justified without quality issues in current system

**Implementation Notes**:
- Keep `sentence-transformers==2.2.2` in requirements.txt
- Maintain 384-dimensional embeddings (no Qdrant collection schema changes)
- Environment variable `EMBEDDING_MODEL` allows future model swapping if needed
- Update embedding scripts to use Qdrant Cloud endpoints instead of local Docker

### 2. Current Codebase Structure

**Question**: What is the existing backend structure and embedding pipeline?

**Findings**: Well-structured FastAPI backend with clear separation of concerns

**Current Structure**:
```
backend/
├── main.py                          # FastAPI entry point
├── src/
│   ├── api/
│   │   └── chatbot_router.py        # API endpoints (chat, embeddings, health)
│   ├── models/
│   │   ├── embedding.py             # VectorEmbedding, TextbookContent, ConversationSession
│   │   ├── query.py                 # UserQuery model
│   │   └── response.py              # RAGResponse model
│   ├── services/
│   │   ├── qdrant_service.py        # Vector DB operations
│   │   └── rag_service.py           # RAG pipeline with Gemini/OpenAI LLMs
│   └── utils/
│       ├── text_splitter.py         # MarkdownTextSplitter with chunking
│       ├── content_processor.py     # Content preparation utilities
│       └── metrics.py               # Performance monitoring
├── scripts/
│   └── index_content.py             # Document indexing script
└── Dockerfile
```

**Current Technology Stack**:
- **Web Framework**: FastAPI 0.104.1 + Uvicorn 0.24.0
- **Vector DB**: qdrant-client 1.7.0 (currently pointing to local Docker)
- **Embeddings**: sentence-transformers 2.2.2 (`all-MiniLM-L6-v2`)
- **LLMs**:
  - Primary: Google Gemini (`google-generativeai>=0.3.0`)
  - Fallback: OpenAI (`openai==1.3.7`)
- **Configuration**: python-dotenv 1.0.0 with `.env` files

**Current Qdrant Configuration**:
- **Connection**: `QDRANT_HOST=localhost`, `QDRANT_PORT=6333`
- **Collection**: Fixed 384-dimensional vectors with COSINE distance
- **Fallback**: Local persistent storage at `/app/qdrant_local` if server unavailable
- **Docker Compose**: Single qdrant service exposing ports 6333 (REST), 6334 (gRPC)

**Migration Impact**:
- **Remove**: `docker-compose.yml` Qdrant service
- **Remove**: Local fallback storage logic in `qdrant_service.py`
- **Update**: Environment variables to use `QDRANT_CLUSTER_ENDPOINT` and `QDRANT_API_KEY`
- **Update**: `scripts/index_content.py` to connect to cloud cluster
- **Keep**: All data models, text splitters, API routes unchanged
- **Keep**: sentence-transformers embedding generation logic

### 3. OpenAI Agents SDK Integration Strategy

**Question**: How should we integrate OpenAI Agents SDK given existing RAG service architecture?

**Decision**: Replace current LLM generation in `rag_service.py` with OpenAI Agents SDK agent

**Rationale**:
- **Current Limitation**: Direct LLM API calls lack conversation management, tool use, and structured agent patterns
- **Agent Benefits**: SDK provides session management, instruction-based behavior, and extensibility for future tools
- **Single-Agent Pattern**: Textbook chatbot fits single-agent architecture (confirmed in spec clarifications)
- **Custom Client Support**: SDK's `OpenAIChatCompletionsModel` with `AsyncOpenAI` enables Gemini integration without LiteLLM

**Implementation Approach**:
1. **Create Agent Module**: `backend/src/agents/chatbot.py` with textbook-specific instructions
2. **Replace RAG Generation**: Refactor `rag_service.py` to use `Runner.run()` instead of direct LLM calls
3. **Session Management**: Leverage SDK's built-in conversation context instead of custom `ConversationSession` model
4. **Model Configuration**: Create `models.py` with factory functions for OpenAI and Gemini clients
5. **Qdrant Integration**: Agent retrieves relevant textbook content from Qdrant, includes in context

**Code Pattern**:
```python
# backend/src/agents/models.py
from openai import AsyncOpenAI
from agents import OpenAIChatCompletionsModel
import os

def get_gemini_model():
    client = AsyncOpenAI(
        base_url=os.getenv("GEMINI_BASE_URL"),
        api_key=os.getenv("GEMINI_API_KEY")
    )
    return OpenAIChatCompletionsModel(
        model=os.getenv("MODEL_NAME", "gemini-2.5-flash-preview"),
        openai_client=client
    )

# backend/src/agents/chatbot.py
from agents import Agent, Runner
from .models import get_gemini_model

async def create_textbook_agent():
    return Agent(
        name="Textbook Assistant",
        instructions="You are a helpful assistant for a Physical AI & Humanoid Robotics textbook...",
        model=get_gemini_model()
    )

async def process_query(query: str, context_docs: list[str]):
    agent = await create_textbook_agent()
    prompt = f"Context:\\n{context_docs}\\n\\nQuestion: {query}"
    result = await Runner.run(agent, input=prompt)
    return result.final_output
```

**Environment Variables Added**:
- `GEMINI_BASE_URL`: Gemini API endpoint
- `GEMINI_API_KEY`: Gemini authentication key
- `MODEL_NAME`: Model identifier (e.g., "gemini-2.5-flash-preview")
- `OPENAI_API_KEY`: For OpenAI models (if used instead of Gemini)

**Backward Compatibility**:
- API routes remain unchanged (`POST /api/v1/chat`)
- Response format (`RAGResponse`) stays the same
- Frontend integration unaffected

### 4. Qdrant Cloud Migration Best Practices

**Question**: What are best practices for migrating from local Docker Qdrant to Qdrant Cloud?

**Decision**: Direct cutover with environment variable switch

**Best Practices Applied**:
1. **Collection Schema Preservation**:
   - Keep 384-dimensional vectors (matches current embeddings)
   - Maintain COSINE distance metric
   - Preserve payload structure (title, content, page, section_header)
   - Use same collection name for continuity

2. **Connection Pattern**:
   ```python
   from qdrant_client import QdrantClient
   import os

   def get_qdrant_client():
       return QdrantClient(
           url=os.environ["QDRANT_CLUSTER_ENDPOINT"],
           api_key=os.environ["QDRANT_API_KEY"],
           timeout=10.0  # Reasonable timeout for cloud
       )
   ```

3. **Environment Validation**:
   - Fail fast on startup if `QDRANT_CLUSTER_ENDPOINT` or `QDRANT_API_KEY` missing
   - Create `scripts/validate_env.py` for pre-flight checks
   - Update `.env.example` with cloud-specific variables

4. **Data Migration**:
   - Option A: Re-run `scripts/index_content.py` pointing to Qdrant Cloud (fresh start)
   - Option B: Export from local, import to cloud using Qdrant snapshot API (if preserving existing data)
   - **Recommendation**: Fresh indexing (clean slate, validates pipeline)

5. **Testing Strategy**:
   - Integration test: `test_qdrant_cloud.py` with real cloud cluster (requires test credentials)
   - Health check endpoint validates connectivity on startup
   - Metrics endpoint monitors query latency and error rates

6. **Rollback Plan**:
   - Keep Docker Compose configuration in git history (commented out)
   - Document rollback procedure in migration guide
   - Maintain separate Qdrant Cloud cluster for staging/testing

### 5. Dependencies to Update

**Current vs. Target Dependencies**:

| Package | Current | Target | Reason |
|---------|---------|--------|--------|
| `qdrant-client` | 1.7.0 | 1.7.0+ | Already compatible with Qdrant Cloud |
| `openai` | 1.3.7 | 1.50.0+ | Need AsyncOpenAI for custom clients |
| `openai-agents-python` | - | 0.2.9+ | **NEW** - Agent framework |
| `sentence-transformers` | 2.2.2 | 2.2.2 | Keep unchanged (no migration) |
| `fastapi` | 0.104.1 | 0.104.1+ | Already adequate |
| `google-generativeai` | 0.3.0+ | 0.3.0+ | Keep for direct Gemini access (legacy compat) |

**New Dependencies**:
- `openai-agents-python>=0.2.9` - Agent orchestration framework
- `pytest-asyncio` - Testing async agent workflows

**Deprecated**:
- Docker Compose for Qdrant (local Docker removed)
- Local fallback storage logic (cloud-only)

## Implementation Recommendations

### Phase 1 Priority Changes (Based on Research)

1. **Update Connection Layer First** (backend/src/services/qdrant_service.py):
   - Remove local fallback logic
   - Implement cloud-only client with env var validation
   - Add connection health check on startup

2. **Refactor RAG Service** (backend/src/services/rag_service.py):
   - Extract LLM generation to agent module
   - Implement OpenAI Agents SDK patterns
   - Maintain existing Qdrant search logic

3. **Create Agent Module** (backend/src/agents/):
   - Implement chatbot agent with textbook instructions
   - Create model factory for OpenAI/Gemini clients
   - Wire agent into RAG pipeline

4. **Update Embedding Pipeline** (backend/scripts/index_content.py):
   - Point to Qdrant Cloud cluster
   - Add progress tracking for large batches
   - Implement error recovery (skip failed docs, continue)

5. **Environment Configuration**:
   - Update `.env.example` with all required cloud variables
   - Create `scripts/validate_env.py` for pre-flight checks
   - Document migration in quickstart.md

### Testing Strategy

1. **Unit Tests**:
   - Test env var validation logic
   - Test agent initialization with mock clients
   - Test embedding generation (unchanged)

2. **Integration Tests**:
   - Test Qdrant Cloud connectivity (requires test cluster)
   - Test agent workflow end-to-end
   - Test vector search and retrieval

3. **Migration Validation**:
   - Run indexing script against Qdrant Cloud
   - Verify collection schema matches expectations
   - Test sample queries for quality

### Performance Considerations

- **Qdrant Cloud Latency**: Expect 50-200ms baseline network latency vs <10ms local
- **Agent SDK Overhead**: Minimal (<50ms) for conversation management
- **Embedding Generation**: Unchanged (local, no network calls)
- **Total Query Time**: Should stay under 30-second target (spec SC-003)

### Security Checklist

- ✅ No hardcoded API keys (all env vars)
- ✅ TLS encryption (Qdrant Cloud uses HTTPS)
- ✅ API key rotation strategy documented
- ✅ Secrets excluded from version control (.gitignore .env)
- ✅ Minimal permissions on API keys (read/write collections only)

## Conclusion

Research confirms feasibility of migration with minimal disruption. Current architecture is well-designed for this refactor. Key takeaways:

1. **Keep sentence-transformers** - No embedding model change needed
2. **Direct SDK integration** - Replace LLM calls with agent patterns
3. **Clean cutover** - Remove Docker, use cloud-only configuration
4. **Fresh indexing recommended** - Simplest migration path
5. **Existing structure preserved** - Data models, API routes unchanged

**Next Steps**: Proceed to Phase 1 (data model & contracts) with full context of current implementation.
