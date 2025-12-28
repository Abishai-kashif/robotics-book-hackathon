# Implementation Plan: Backend Tech Stack Refactor - OpenAI Agents SDK + Qdrant Cloud

**Branch**: `007-backend-tech-stack-refactor` | **Date**: 2025-12-28 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/007-backend-tech-stack-refactor/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Migrate backend architecture from local Qdrant Docker to Qdrant Cloud managed service, adopt OpenAI Agents SDK for conversational AI agent orchestration, and support multi-model configuration (OpenAI, Gemini) through environment variable-based credentials. Primary technical approach: Replace local vector database with cloud-hosted solution, implement single-agent chatbot using SDK custom client pattern for model flexibility, refactor embedding scripts for cloud storage.

## Technical Context

**Language/Version**: Python 3.10+ (async/await support required for OpenAI Agents SDK)
**Primary Dependencies**:
- `openai-agents-python` (v0.2.9+) - Agent orchestration framework
- `qdrant-client` - Vector database client for Qdrant Cloud
- `openai` - AsyncOpenAI client for custom provider support (Gemini)
- `python-dotenv` - Environment variable loading from .env files
- NEEDS CLARIFICATION: Embedding model library (OpenAI embeddings API vs sentence-transformers vs other)

**Storage**: Qdrant Cloud (managed vector database), collections for textbook content embeddings
**Testing**: pytest with async support (pytest-asyncio), integration tests for Qdrant Cloud connectivity, agent workflow tests
**Target Platform**: Linux server (Ubuntu 22.04), cloud deployable (AWS/Azure), support for local development with .env files
**Project Type**: Web backend with conversational AI agent and embedding pipeline
**Performance Goals**:
- Vector search latency <500ms for 95% of requests
- Agent query response <30 seconds end-to-end
- Embedding generation ≥10 documents/minute

**Constraints**:
- Zero hardcoded secrets (SEC-001)
- No local Qdrant Docker instances (TC-001)
- Environment variables exclusively for config (TC-003)
- TLS/HTTPS for all external connections (TC-007)

**Scale/Scope**:
- Textbook chatbot (single agent)
- Qdrant Cloud collections (textbook documents)
- Support for multiple LLM providers (OpenAI, Gemini)
- NEEDS CLARIFICATION: Current codebase structure and existing embedding pipeline details

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Physical AI & Humanoid Robotics Constitution Compliance

| Principle | Relevance | Compliance Status |
|-----------|-----------|-------------------|
| **I. Physical AI First** | ⚠️ PARTIAL | Backend infrastructure refactor supports textbook content delivery for Physical AI education but doesn't directly implement physical robot control |
| **II. ROS 2 Integration** | ✅ NOT APPLICABLE | Backend database and agent framework migration; ROS 2 is content domain, not backend infrastructure concern |
| **III. Simulation-to-Reality** | ✅ NOT APPLICABLE | Infrastructure refactor; simulation content is served by this backend but not implemented here |
| **IV. Multi-Platform Compatibility** | ✅ COMPLIANT | Cloud-based Qdrant and env-var config support both cloud and local deployment scenarios per constitution requirement |
| **V. Vision-Language-Action** | ⚠️ PARTIAL | Conversational AI agent (language) supports VLA content delivery but doesn't implement robot action sequences |
| **VI. Humanoid-Centric Design** | ✅ NOT APPLICABLE | Backend infrastructure; humanoid robotics is content domain |
| **VII. Conversational AI Integration** | ✅ COMPLIANT | OpenAI Agents SDK implementation directly supports conversational AI for textbook chatbot per constitution requirement |
| **VIII. Hardware-Aware Delivery** | ✅ COMPLIANT | Cloud deployment (Qdrant Cloud) and local development (.env config) support different hardware configurations as required |

**Assessment**: PASS - This is a backend infrastructure refactor that enables the textbook's educational content delivery. The constitution principles apply to the **content** (what is taught), not the **infrastructure** (how content is delivered). Backend must support cloud/local deployment (✅), conversational AI capabilities (✅), and multi-platform scenarios (✅). Physical AI principles and ROS 2 integration apply to the educational material served by this backend, not the backend technology stack itself.

**Justification**: Backend refactor is a supporting system for educational content. Constitution compliance is evaluated at the content layer (modules teaching ROS 2, physical AI, humanoid control) rather than infrastructure layer (database, agents, embeddings). This refactor enhances content delivery capability without changing what is taught.

## Project Structure

### Documentation (this feature)

```text
specs/007-backend-tech-stack-refactor/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Backend structure (web application)
backend/
├── src/
│   ├── config/
│   │   ├── __init__.py
│   │   └── env.py              # Environment variable validation
│   ├── database/
│   │   ├── __init__.py
│   │   └── qdrant_client.py    # Qdrant Cloud client initialization
│   ├── agents/
│   │   ├── __init__.py
│   │   ├── chatbot.py          # Main textbook chatbot agent
│   │   └── models.py           # Model client configurations (OpenAI, Gemini)
│   ├── embeddings/
│   │   ├── __init__.py
│   │   ├── generator.py        # Embedding generation logic
│   │   └── uploader.py         # Qdrant Cloud upload
│   └── api/
│       ├── __init__.py
│       └── routes.py           # FastAPI routes for agent interaction
├── tests/
│   ├── integration/
│   │   ├── test_qdrant_cloud.py
│   │   └── test_agent_workflow.py
│   └── unit/
│       ├── test_config.py
│       ├── test_agents.py
│       └── test_embeddings.py
├── scripts/
│   ├── embed_textbook.py      # Embedding pipeline script
│   └── validate_env.py         # Environment variable checker
├── .env.example                # Template for required env vars
└── requirements.txt

frontend/
├── src/
│   ├── components/
│   │   └── ChatInterface.tsx   # Chatbot UI component
│   └── services/
│       └── agent.ts            # Agent API client
└── tests/
```

**Structure Decision**: Web application structure chosen because feature involves both backend infrastructure (database, agents, embeddings) and frontend chatbot interface. Backend handles Qdrant Cloud connection, OpenAI Agents SDK integration, and embedding generation. Frontend provides user-facing chat interface for textbook Q&A.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

*No violations requiring justification. Constitution compliance achieved through proper layering: infrastructure refactor supports educational content delivery without changing pedagogical principles.*
