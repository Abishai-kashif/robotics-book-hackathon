---
id: 003
title: Backend tech stack implementation plan
stage: plan
date: 2025-12-28
surface: agent
model: claude-sonnet-4-5
feature: 007-backend-tech-stack-refactor
branch: 007-backend-tech-stack-refactor
user: system
command: /sp.plan
labels: ["planning", "architecture", "qdrant-cloud", "openai-agents-sdk", "data-model", "api-contracts"]
links:
  spec: specs/007-backend-tech-stack-refactor/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/007-backend-tech-stack-refactor/plan.md
  - specs/007-backend-tech-stack-refactor/research.md
  - specs/007-backend-tech-stack-refactor/data-model.md
  - specs/007-backend-tech-stack-refactor/contracts/api-spec.yaml
  - specs/007-backend-tech-stack-refactor/quickstart.md
  - CLAUDE.md (context updated with Python 3.10+ and Qdrant Cloud)
tests:
  - Constitution compliance check (PASSED)
  - Technical context validation (all unknowns resolved)
  - API contract specification (OpenAPI 3.0.3)
---

## Prompt

/sp.plan

## Response snapshot

Successfully completed implementation planning for backend tech stack refactor migration to Qdrant Cloud and OpenAI Agents SDK:

### Phase 0: Research & Resolution

**Codebase Exploration** (using Explore agent):
- Current structure: Well-architected FastAPI backend with clear separation (api, services, models, utils)
- Embedding library: sentence-transformers 2.2.2 with all-MiniLM-L6-v2 (384-dim)
- Current stack: FastAPI, qdrant-client 1.7.0 (local Docker), Gemini primary + OpenAI fallback
- Qdrant connection: localhost:6333 with fallback to local persistent storage
- Indexing pipeline: scripts/index_content.py with MarkdownTextSplitter

**Key Decisions Documented in research.md**:
1. **Embedding Model**: Keep sentence-transformers (no migration needed, cost-effective, proven)
2. **Integration Strategy**: Replace LLM calls in rag_service.py with OpenAI Agents SDK
3. **Migration Approach**: Direct cutover with fresh indexing (cleanest path)
4. **Gemini Integration**: Custom AsyncOpenAI client (no LiteLLM dependency per clarifications)

### Phase 1: Design & Contracts

**Data Model (data-model.md)**: 7 core entities defined with relationships, state management, validation rules
- EnvironmentConfig (new): Centralized env var validation
- VectorEmbedding (unchanged): 384-dim embeddings, Qdrant Cloud storage
- TextbookContent (unchanged): Chunked documents with metadata
- AgentConfig (new): LLM client settings for SDK
- ConversationSession (refactored): SDK session management
- UserQuery (unchanged): Incoming request model
- RAGResponse (unchanged): Response with sources and confidence

**API Contracts (contracts/api-spec.yaml)**: OpenAPI 3.0.3 specification
- POST /api/v1/chat - RAG query processing
- POST /api/v1/embeddings - Content indexing
- GET /api/v1/health - Service health check
- GET /api/v1/metrics - Performance monitoring
- Complete request/response schemas with validation rules
- Error taxonomy with machine-readable codes

**Quickstart Guide (quickstart.md)**: 10-step developer onboarding
1. Provision Qdrant Cloud cluster
2. Configure environment variables (.env)
3. Install dependencies (added openai-agents-python, updated openai)
4. Validate environment (scripts/validate_env.py)
5. Create Qdrant collection (384-dim COSINE)
6. Index textbook content (scripts/index_content.py)
7. Test agent workflow (pytest integration tests)
8. Start development server (uvicorn)
9. Test API endpoints (curl examples)
10. Deploy to production (Docker/cloud options)

### Constitution Compliance

**Assessment**: PASS - Backend infrastructure refactor supports educational content delivery

**Rationale**: Constitution principles apply to **content** (what is taught: ROS 2, Physical AI, humanoid robotics), not **infrastructure** (how content is delivered: database, agents, embeddings). This refactor:
- ✅ Supports cloud/local deployment (Principle IV, VIII)
- ✅ Enables conversational AI for education (Principle VII)
- ✅ Multi-platform compatibility maintained (Principle IV)

**Key Insight**: Infrastructure layer is supporting system; ROS 2/Physical AI compliance evaluated at content layer (textbook modules, not backend stack).

### Project Structure Decisions

**Backend Structure**: Web application pattern (backend + frontend separation)
- backend/src/config/ - Environment validation
- backend/src/database/ - Qdrant Cloud client
- backend/src/agents/ - Chatbot agent & model factories
- backend/src/embeddings/ - Generation & upload
- backend/src/api/ - FastAPI routes
- backend/scripts/ - Indexing & validation utilities

**Migration Strategy**:
- Remove: docker-compose.yml Qdrant service, local fallback logic
- Update: Environment variables, Qdrant client initialization
- Keep: All data models, API routes, text splitters unchanged
- Add: OpenAI Agents SDK integration, agent module, model factories

### Technical Decisions

1. **Sentence Transformers Retained**: No embedding model change (migration cost not justified, current system performs well)
2. **Single-Agent Architecture**: Confirmed per spec clarifications (no handoffs needed for textbook chatbot)
3. **Custom Client Pattern**: Gemini via AsyncOpenAI (base_url + api_key) eliminates LiteLLM dependency
4. **Fresh Indexing**: Recommended over export/import (clean slate, validates pipeline)
5. **Cloud-Only Configuration**: No local fallback (simplifies code, enforces hard constraint)

### Performance & Security

**Performance Goals**:
- Vector search latency <500ms (p95)
- Agent response <30s end-to-end
- Embedding generation ≥10 docs/min

**Security Measures**:
- Zero hardcoded secrets (all env vars)
- TLS encryption (Qdrant Cloud HTTPS)
- Input validation middleware
- API key rotation strategy documented

### Dependencies

**New**:
- openai-agents-python>=0.2.9 - Agent framework
- openai>=1.50.0 - AsyncOpenAI for custom clients
- pytest-asyncio - Async testing

**Updated**:
- qdrant-client>=1.7.0 - Already compatible

**Removed**:
- docker-compose Qdrant service
- Local fallback storage logic

## Outcome

- ✅ Impact: Complete implementation plan with phase 0 research (resolved unknowns), phase 1 design (data model, API contracts, quickstart); constitution compliance validated; no ADRs needed (refactor, not new architecture)
- 🧪 Tests: Constitution check PASSED; Technical context validated (no NEEDS CLARIFICATION remaining); API contract spec complete (OpenAPI 3.0.3)
- 📁 Files: Created plan.md, research.md, data-model.md, contracts/api-spec.yaml, quickstart.md; Updated CLAUDE.md (agent context)
- 🔁 Next prompts: `/sp.tasks` to generate implementation tasks from plan, or review planning artifacts for refinement
- 🧠 Reflection: Planning phase successfully completed with comprehensive research (explored existing codebase), clear design decisions (keep embeddings, SDK integration, cloud-only), and detailed contracts; ready for task breakdown

## Evaluation notes (flywheel)

- Failure modes observed: None - workflow executed successfully with Explore agent for codebase understanding
- Graders run and results (PASS/FAIL): Constitution compliance PASS (infrastructure layer supports content delivery); Technical context validation PASS (all unknowns resolved); API contract completeness PASS (full OpenAPI spec)
- Prompt variant (if applicable): Standard /sp.plan workflow with Phase 0 research (Explore agent) and Phase 1 design (data model, contracts, quickstart)
- Next experiment (smallest change to try): Proceed to /sp.tasks for implementation breakdown; consider ADR for "Single-Agent vs Multi-Agent Architecture" decision (meets significance test: impact, alternatives, scope)
