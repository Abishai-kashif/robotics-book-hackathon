# Data Model: Chatbot Integration with Docusaurus for Textbook RAG

## Entity: UserQuery
**Description**: Represents a natural language query submitted by a student
**Fields**:
- query_id: String (unique identifier for the query)
- content: String (the actual text of the user's question)
- timestamp: DateTime (when the query was submitted)
- user_context: Object (optional context like current page, user session info)
- source_page: String (optional URL/path of the current textbook page)

**Validation rules**:
- content must be between 5 and 1000 characters
- query_id must be unique
- timestamp must be current or recent

## Entity: TextbookContent
**Description**: Represents a chunk of textbook content that has been processed for vector search
**Fields**:
- content_id: String (unique identifier for the content chunk)
- title: String (title of the content section)
- body: String (the actual text content)
- source_path: String (path to the original content in the textbook)
- embedding: Array<float> (vector representation of the content)
- metadata: Object (additional information like module, chapter, etc.)

**Validation rules**:
- content_id must be unique
- body must not be empty
- embedding must have a fixed dimension (e.g., 384, 768, or 1536 depending on model)
- source_path must be a valid textbook content path

## Entity: VectorEmbedding
**Description**: Numerical representation of textbook content for semantic search
**Fields**:
- embedding_id: String (unique identifier)
- content_id: String (reference to the TextbookContent)
- vector: Array<float> (the actual embedding values)
- model_used: String (identifier of the embedding model used)
- created_at: DateTime (timestamp of when the embedding was generated)

**Validation rules**:
- embedding_id must be unique
- content_id must reference an existing TextbookContent
- vector must have the correct dimensionality
- model_used must be a valid model identifier

## Entity: RAGResponse
**Description**: The generated response that combines retrieved content with conversational AI
**Fields**:
- response_id: String (unique identifier for the response)
- query_id: String (reference to the original user query)
- answer: String (the generated answer text)
- sources: Array<Object> (list of source citations)
- confidence_score: Float (confidence level of the response)
- timestamp: DateTime (when the response was generated)

**Validation rules**:
- response_id must be unique
- query_id must reference an existing UserQuery
- answer must not be empty
- confidence_score must be between 0.0 and 1.0
- sources must contain valid citations

## Entity: SourceCitation
**Description**: Reference to specific parts of the textbook content that support the response
**Fields**:
- citation_id: String (unique identifier)
- response_id: String (reference to the RAGResponse)
- content_id: String (reference to the TextbookContent)
- text_snippet: String (relevant excerpt from the source)
- source_path: String (path to the original content)
- page_reference: String (page number or section reference)

**Validation rules**:
- citation_id must be unique
- response_id must reference an existing RAGResponse
- content_id must reference an existing TextbookContent
- text_snippet must not be empty

## Entity: ConversationSession
**Description**: Maintains context for multi-turn conversations
**Fields**:
- session_id: String (unique identifier for the session)
- user_id: String (optional identifier for the user)
- created_at: DateTime (when the session started)
- last_activity: DateTime (when the last interaction occurred)
- history: Array<Object> (conversation history with queries and responses)

**Validation rules**:
- session_id must be unique
- created_at must be before or equal to last_activity
- history items must be valid query-response pairs

## Relationships

1. **UserQuery** → **RAGResponse**: One-to-many (one query can have multiple responses in a session)
2. **TextbookContent** → **VectorEmbedding**: One-to-one (each content has one embedding)
3. **RAGResponse** → **SourceCitation**: One-to-many (one response can cite multiple sources)
4. **ConversationSession** → **UserQuery**: One-to-many (one session can contain multiple queries)

## State Transitions

**ConversationSession**:
- Created → Active (when first query is submitted)
- Active → Inactive (after period of inactivity or explicit session end)
- Inactive → Archived (after extended inactivity, may be cleaned up)