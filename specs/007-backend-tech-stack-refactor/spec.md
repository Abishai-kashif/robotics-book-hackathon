# Feature Specification: Backend Tech Stack Refactor - OpenAI Agents SDK + Qdrant Cloud

**Feature Branch**: `007-backend-tech-stack-refactor`
**Created**: 2025-12-28
**Status**: Draft
**Input**: User description: "Refactor backend: Adopt OpenAI Agents SDK + Gemini, migrate to Qdrant Cloud with env-based config"

## Clarifications

### Session 2025-12-28

- Q: Should agent handoffs (multi-agent delegation) be included in the architecture? → A: No - single-agent architecture with multiple capabilities is sufficient for textbook chatbot workflow
- Q: Should tool implementation details be specified in functional requirements? → A: No - requirements should be outcome-focused (what agents need to accomplish: retrieve content, search semantically) rather than implementation-focused (how they implement it: function tools)
- Q: Should LiteLLM be used as a dependency for Gemini integration? → A: No - Gemini can be configured directly via OpenAI Agents SDK custom client support (AsyncOpenAI with base_url and api_key), eliminating the additional dependency
- Q: Do embedding scripts need to use OpenAI Agents SDK patterns? → A: No - embedding scripts are straightforward batch processes (read, generate, store) and don't require agent orchestration framework

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Qdrant Cloud Migration (Priority: P1)

The development team migrates from local Qdrant Docker instances to Qdrant Cloud, ensuring the vector database is accessible through managed cloud infrastructure with proper authentication.

**Why this priority**: This is foundational - all vector search functionality depends on having a working, production-ready vector database. Without this migration, the system cannot operate in a cloud environment.

**Independent Test**: Can be fully tested by instantiating the Qdrant client with environment variables (`QDRANT_API_KEY`, `QDRANT_CLUSTER_ENDPOINT`) and successfully performing a health check or simple query against the Qdrant Cloud cluster. Delivers a working cloud-based vector database connection.

**Acceptance Scenarios**:

1. **Given** a Qdrant Cloud cluster is provisioned with an API key, **When** the system initializes the Qdrant client using environment variables, **Then** the client successfully connects to the cloud cluster without errors
2. **Given** the Qdrant client is initialized, **When** a test query is executed, **Then** the query returns results from the cloud-hosted collection
3. **Given** environment variables are not set, **When** the system attempts to initialize the Qdrant client, **Then** the system fails gracefully with a clear error message indicating missing credentials
4. **Given** invalid credentials are provided, **When** the system attempts to connect, **Then** authentication fails with an appropriate error message

---

### User Story 2 - Environment-Based Configuration (Priority: P1)

Developers and operators configure the system using environment variables for all sensitive credentials, ensuring no secrets are hardcoded in the codebase.

**Why this priority**: Security-critical requirement. Hardcoded secrets create significant security vulnerabilities and violate standard deployment practices. This must be in place before any production deployment.

**Independent Test**: Can be fully tested by verifying that all database connections and API clients read credentials exclusively from environment variables, and that the codebase contains no hardcoded secrets. Delivers secure credential management.

**Acceptance Scenarios**:

1. **Given** required environment variables are set in `.env` or system environment, **When** the application starts, **Then** all services initialize with credentials from environment variables only
2. **Given** a code review is performed, **When** searching for hardcoded credentials, **Then** no API keys, passwords, or cluster endpoints are found in source code
3. **Given** environment variables are changed, **When** the application is restarted, **Then** the new credentials are used without code changes
4. **Given** a deployment to a new environment, **When** only environment variables are updated, **Then** the system connects to the correct resources

---

### User Story 3 - OpenAI Agents SDK Integration (Priority: P2)

The development team adopts the OpenAI Agents SDK for building and orchestrating AI agents, replacing any previous agent frameworks with the provider-agnostic SDK.

**Why this priority**: Core functionality upgrade that enables better agent orchestration. Can be implemented after database migration is complete since it depends on having vector storage operational.

**Independent Test**: Can be fully tested by creating an agent using the OpenAI Agents SDK, configuring it with instructions, and successfully running it to process a test query and retrieve relevant textbook content. Delivers a working agent framework.

**Acceptance Scenarios**:

1. **Given** the OpenAI Agents SDK is installed, **When** a developer creates an agent with instructions, **Then** the agent initializes successfully
2. **Given** an agent is created, **When** the agent is run with an input prompt, **Then** the agent processes the prompt and returns a response
3. **Given** an agent receives a user question, **When** the agent needs textbook content, **Then** the agent successfully retrieves relevant content from Qdrant Cloud
4. **Given** the agent is processing a query, **When** external operations are needed (search, retrieval), **Then** operations complete successfully and results inform the agent's response

---

### User Story 4 - Gemini Model Integration (Priority: P2)

The development team configures agents to use Gemini models through the OpenAI Agents SDK's custom client support, enabling access to Google's large language models without additional dependencies.

**Why this priority**: Extends model capabilities and provides model flexibility. Should follow agent SDK integration since it builds on that foundation.

**Independent Test**: Can be fully tested by creating an agent configured with a Gemini model using a custom OpenAI client, running a test query, and verifying the response comes from the Gemini API. Delivers multi-model support.

**Acceptance Scenarios**:

1. **Given** Gemini API credentials are configured via environment variables (`GEMINI_BASE_URL`, `GEMINI_API_KEY`, `MODEL_NAME`), **When** an agent is initialized with a custom client pointing to Gemini, **Then** the agent initializes successfully
2. **Given** an agent configured for Gemini, **When** the agent processes a query, **Then** the response is generated by the Gemini model
3. **Given** the system can use multiple models, **When** switching between OpenAI and Gemini, **Then** configuration changes only require updating environment variables and client initialization
4. **Given** Gemini API credentials are missing or invalid, **When** the agent attempts to initialize, **Then** the system fails gracefully with a clear error message

---

### User Story 5 - Embedding Script Modernization (Priority: P3)

Developers update embedding generation scripts to store embeddings in Qdrant Cloud using the new environment-based configuration.

**Why this priority**: Enhancement to existing functionality. Can be done after core infrastructure (Qdrant Cloud) is in place since it depends on cloud database availability.

**Independent Test**: Can be fully tested by running an embedding script that processes textbook documents, generates embeddings, and stores them in Qdrant Cloud collections. Delivers updated embedding pipeline.

**Acceptance Scenarios**:

1. **Given** documents are provided as input, **When** the embedding script runs, **Then** embeddings are generated for all documents
2. **Given** embeddings are generated, **When** they are stored in Qdrant Cloud, **Then** they are persisted in the specified collection
3. **Given** embeddings exist in Qdrant Cloud, **When** a similarity search is performed, **Then** relevant documents are retrieved based on vector similarity
4. **Given** the embedding script encounters errors, **When** processing fails, **Then** clear error messages indicate what went wrong

---

### Edge Cases

- What happens when Qdrant Cloud API is temporarily unavailable?
  - System should implement retry logic with exponential backoff
  - Graceful degradation: return cached results or friendly error message

- What happens when environment variables are partially set (e.g., API key but no endpoint)?
  - System should validate all required environment variables at startup
  - Fail fast with clear message listing missing variables

- What happens when API rate limits are exceeded?
  - System should detect rate limit errors
  - Implement appropriate backoff and queuing strategies

- What happens when an agent workflow fails mid-execution?
  - Agent SDK should maintain session state
  - Error handling should allow graceful recovery or rollback

- What happens when embedding generation fails for some documents but not others?
  - System should continue processing remaining documents
  - Log failures for review and retry
  - Provide summary of successes and failures

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST connect to Qdrant Cloud using a client initialized exclusively with `QDRANT_API_KEY` and `QDRANT_CLUSTER_ENDPOINT` environment variables
- **FR-002**: System MUST NOT contain any hardcoded API keys, cluster endpoints, passwords, or other secrets in source code
- **FR-003**: System MUST remove all local Qdrant Docker setup code and configuration
- **FR-004**: System MUST use the OpenAI Agents SDK (`openai-agents-python`) as the framework for building and running AI agents
- **FR-005**: System MUST support Gemini models through custom OpenAI client configuration with environment variables (`GEMINI_BASE_URL`, `GEMINI_API_KEY`, `MODEL_NAME`)
- **FR-006**: System MUST allow agents to retrieve relevant textbook content from Qdrant Cloud based on user queries
- **FR-007**: System MUST perform vector similarity searches to find semantically related content
- **FR-008**: System MUST maintain conversation context across multi-turn interactions with users
- **FR-009**: System MUST validate all required environment variables at application startup
- **FR-010**: System MUST fail gracefully with informative error messages when environment variables are missing or invalid
- **FR-011**: System MUST read credentials from `.env` files in development and from system environment variables in production
- **FR-012**: Embedding scripts MUST generate vectors and store them in Qdrant Cloud collections
- **FR-013**: System MUST support vector similarity search operations through the Qdrant Cloud client
- **FR-014**: System MUST implement appropriate error handling for network failures, authentication errors, and API rate limits

### Key Entities

- **Agent**: An LLM-powered conversational entity with instructions and model configuration that processes user queries and retrieves relevant textbook content
- **Qdrant Collection**: A vector database collection storing embeddings with associated metadata for similarity search operations
- **Vector Embedding**: High-dimensional numerical representation of text or other content used for semantic similarity search
- **Environment Configuration**: Set of key-value pairs defining API keys, cluster endpoints, and other operational parameters sourced from environment variables
- **Conversation Context**: Message history and state maintained across multi-turn interactions with users
- **Custom Model Client**: OpenAI-compatible client configuration that enables connecting to alternative LLM providers (e.g., Gemini)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Vector database operations complete successfully against Qdrant Cloud with query latency under 500ms for 95% of requests
- **SC-002**: All secrets and credentials are externalized to environment variables with zero hardcoded values in codebase (verifiable through code scanning)
- **SC-003**: Agent processes user queries end-to-end, from input to final response with relevant textbook content, in under 30 seconds for typical queries
- **SC-004**: System successfully initializes in both development and production environments using only environment variable configuration
- **SC-005**: Embedding generation processes documents at a rate of at least 10 documents per minute
- **SC-006**: Conversation context is maintained across multi-turn interactions without loss of relevant history
- **SC-007**: System handles authentication failures and network errors gracefully without crashes, returning appropriate error messages to users
- **SC-008**: Development team can switch between different LLM models (OpenAI, Gemini) by changing environment variables only, without code refactoring

## Scope & Dependencies *(mandatory)*

### In Scope

- Migration of Qdrant database from local Docker to Qdrant Cloud managed service
- Implementation of environment variable-based configuration for all credentials
- Integration of OpenAI Agents SDK for agent orchestration
- Configuration of Gemini model support via custom OpenAI client
- Refactoring of embedding generation scripts to use Qdrant Cloud
- Updating all database client initialization code
- Removing Docker Compose configurations for local Qdrant
- Documentation of environment variable requirements
- Testing of agent workflows with both OpenAI and Gemini models

### Out of Scope

- Changes to vector embedding model or dimensions (keep existing embedding strategy)
- Migration or modification of existing document content
- UI/UX changes to frontend applications
- Changes to authentication/authorization for end users
- Performance optimization beyond baseline functionality
- Data backup and disaster recovery strategies (assuming Qdrant Cloud handles this)
- Cost optimization and resource scaling strategies
- Migration of existing vector data (assume fresh start or separate migration script)

### External Dependencies

- **Qdrant Cloud**: Managed vector database service (qdrant.tech)
  - Ownership: External vendor (Qdrant)
  - Required: Active cluster with API key
  - SLA: Per Qdrant Cloud service agreement

- **OpenAI Agents SDK**: Python package for agent orchestration
  - Ownership: OpenAI (github.com/openai/openai-agents-python)
  - Required: Version v0.2.9 or later
  - Status: Open source, actively maintained
  - Note: Supports custom OpenAI-compatible clients for alternative providers

- **Gemini API**: Google's large language model API
  - Ownership: Google
  - Required: API credentials (base URL, API key) for Gemini model access
  - Integration: Via custom OpenAI client (no additional dependencies)
  - SLA: Per Google Cloud terms

- **Python Environment**: Runtime environment
  - Required: Python 3.8+ with pip
  - Ownership: Development team

### Assumptions

- Qdrant Cloud cluster has been provisioned and API keys have been generated
- Team has access to required API credentials (Qdrant, OpenAI, Gemini)
- Existing codebase uses Python as primary language
- Development team has basic familiarity with async/await patterns in Python
- `.env` file support is available via python-dotenv or similar library
- Network connectivity between application and cloud services is stable
- Current vector dimension and distance metric are compatible with Qdrant Cloud
- Team has sufficient API rate limits and quotas for development and testing

## Constraints *(mandatory)*

### Technical Constraints

- **TC-001**: MUST NOT use local Qdrant Docker instances under any circumstances
- **TC-002**: MUST NOT hardcode any credentials, API keys, or endpoints in source code
- **TC-003**: MUST use environment variables exclusively for all configuration secrets
- **TC-004**: MUST use OpenAI Agents SDK as the sole agent framework (no mixing with other frameworks)
- **TC-005**: MUST maintain backward compatibility with existing vector data schema (collection structure, field names)
- **TC-006**: MUST support Python async/await patterns for agent and database operations
- **TC-007**: MUST use HTTPS/TLS for all connections to Qdrant Cloud and API services

### Business Constraints

- **BC-001**: Migration must not disrupt existing functionality beyond planned downtime
- **BC-002**: Team must maintain ability to roll back to previous architecture if critical issues arise
- **BC-003**: Documentation must be updated to reflect new architecture before production deployment

### Security Constraints

- **SEC-001**: All credentials MUST be stored outside version control (in `.env` files or secret management systems)
- **SEC-002**: Qdrant Cloud connections MUST use TLS encryption
- **SEC-003**: API keys MUST have minimum required permissions (principle of least privilege)
- **SEC-004**: Error messages MUST NOT leak sensitive information (credentials, internal paths, stack traces to users)

## Notes & Additional Context *(optional)*

### Authoritative Documentation Sources

Per project requirements, the following authoritative documentation was consulted:

**Qdrant (Vector Database)**
- Library ID: `/llmstxt/qdrant_tech_llms-full_txt`
- Code Snippets: 14,931
- Key Topics: Qdrant Cloud setup, environment variable configuration, client initialization, migration from local Docker
- Benchmark Score: 83.1 (High reputation)

**OpenAI Agents SDK**
- Library ID: `/openai/openai-agents-python`
- Code Snippets: 255
- Version: v0.2.9
- Key Topics: Agent architecture, model configuration, Gemini integration, tool creation, handoffs, sessions
- Benchmark Score: 86.4 (High reputation)

### Technical Notes

1. **Qdrant Client Initialization Pattern**:
   ```python
   from qdrant_client import QdrantClient
   import os

   client = QdrantClient(
       url=os.environ.get("QDRANT_CLUSTER_ENDPOINT"),
       api_key=os.environ.get("QDRANT_API_KEY")
   )
   ```

2. **Agent Creation Pattern with Gemini**:
   ```python
   from openai import AsyncOpenAI
   from agents import Agent, OpenAIChatCompletionsModel
   import os

   # Custom client for Gemini
   client = AsyncOpenAI(
       base_url=os.getenv("GEMINI_BASE_URL"),
       api_key=os.getenv("GEMINI_API_KEY")
   )

   agent = Agent(
       name="Assistant",
       instructions="Your instructions here",
       model=OpenAIChatCompletionsModel(
           model=os.getenv("MODEL_NAME"),
           openai_client=client
       )
   )
   ```

3. **Environment Variables Required**:
   - `QDRANT_CLUSTER_ENDPOINT`: Full URL to Qdrant Cloud cluster
   - `QDRANT_API_KEY`: Authentication key for Qdrant Cloud
   - `OPENAI_API_KEY`: For OpenAI models (if used)
   - `GEMINI_BASE_URL`: Base URL for Gemini API endpoint
   - `GEMINI_API_KEY`: Authentication key for Gemini API
   - `MODEL_NAME`: Model identifier for Gemini (e.g., "gemini-2.5-flash-preview")

### Migration Considerations

- Docker Compose files referencing local Qdrant should be removed or commented out
- Connection string patterns in codebase should be identified and replaced
- Test suites should be updated to work with cloud-based instances (may require test cluster)
- CI/CD pipelines should inject environment variables securely

### Risk Mitigation

- Maintain a rollback branch with working local Docker setup until cloud migration is validated
- Implement comprehensive integration tests before removing local Docker support
- Document all environment variables clearly in README and deployment guides
- Use `.env.example` file to template required configuration without exposing secrets
