● Agent Workflow Flow

  Here's how the agent works in this backend:

  ┌─────────────────────────────────────────────────────────────────────────────────┐
  │                              Agent Workflow                                      │
  └─────────────────────────────────────────────────────────────────────────────────┘

    1. USER QUERY                          2. RAG (Retrieval)
    ┌─────────────────┐                   ┌────────────────────────┐
    │ POST /api/v1/chat │                  │ Generate query embed   │
    │ {content: "..."} │                   │ using sentence-        │
    └────────┬────────┘                   │ transformers           │
             │                            └───────────┬────────────┘
             │                                        │
             ▼                                        ▼
    ┌─────────────────┐                   ┌────────────────────────┐
    │ chatbot_router   │                  │ Search Qdrant Cloud    │
    │ chat_endpoint()  │                  │ for similar content    │
    └────────┬────────┘                   │ (cosine similarity)    │
             │                            └───────────┬────────────┘
             │                                        │
             ▼                                        ▼
    ┌────────────────────────────────────────────────────────────────────────────┐
    │                           RAG SERVICE                                       │
    │  ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────────┐   │
    │  │ Load embedding  │────▶│ Query Qdrant    │────▶│ Get context text    │   │
    │  │ model (384-dim) │     │ Cloud vector DB │     │ from top results    │   │
    │  └─────────────────┘     └─────────────────┘     └─────────────────────┘   │
    └────────────────────────────────────────────────────────────────────────────┘
             │
             │ context_text
             ▼
    ┌────────────────────────────────────────────────────────────────────────────┐
    │                        OPENAI AGENTS SDK                                    │
    │  ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────────┐   │
    │  │ create_textbook │────▶│ Runner.run()    │────▶│ Get final_output    │   │
    │  │ _agent()        │     │ (Gemini via     │     │ (Agent response)    │   │
    │  │                 │     │ custom client)  │     │                     │   │
    │  └─────────────────┘     └─────────────────┘     └─────────────────────┘   │
    │        │                          │                        │                 │
    │        ▼                          ▼                        ▼                 │
    │  ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────────┐   │
    │  │ Model factory   │     │ Agent processes │     │ Return to user      │   │
    │  │ (get_gemini_    │     │ query + context │     │ with answer +       │   │
    │  │ model())        │     │ + instructions  │     │ source citations    │   │
    │  └─────────────────┘     └─────────────────┘     └─────────────────────┘   │
    └────────────────────────────────────────────────────────────────────────────┘
             │
             ▼
    ┌─────────────────┐
    │ Return RAGResponse │
    │ {answer, sources,│
    │  processing_time} │
    └─────────────────┘

  Key Files

  | File                          | Purpose                             |
  |-------------------------------|-------------------------------------|
  | src/api/chatbot_router.py:22  | API endpoint receives query         |
  | src/services/rag_service.py   | Embeds query, searches Qdrant       |
  | src/agents/chatbot.py:40      | Creates agent with instructions     |
  | src/agents/models.py:50       | Factory creates Gemini/OpenAI model |
  | src/database/qdrant_client.py | Connects to Qdrant Cloud            |

  The Flow in Steps

  1. Query → User sends question to /api/v1/chat
  2. Embed → Query converted to 384-dim vector
  3. Search → Vector search in Qdrant Cloud finds similar chunks
  4. Context → Top results become context text
  5. Agent → Gemini model receives: instructions + context + query
  6. Response → result.final_output returned as answer