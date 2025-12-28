---
id: 002
title: Refactor spec remove handoffs litellm
stage: spec
date: 2025-12-28
surface: agent
model: claude-sonnet-4-5
feature: 007-backend-tech-stack-refactor
branch: 007-backend-tech-stack-refactor
user: system
command: /sp.clarify
labels: ["clarification", "refactoring", "handoffs", "litellm", "gemini", "tools", "architecture-simplification"]
links:
  spec: specs/007-backend-tech-stack-refactor/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/007-backend-tech-stack-refactor/spec.md
tests:
  - Specification clarity validation (outcome-focused requirements)
  - Dependency simplification (removed LiteLLM)
---

## Prompt

Refactor the specifications using clarification on handoffs and tools in our conversation and consider this also: Completely remove the litellm as a dependency as we can configure gemini using just agents sdk as shown in the code below:

```python
import asyncio
import os

from openai import AsyncOpenAI

from agents import Agent, OpenAIChatCompletionsModel

GEMINI_BASE_URL= os.getenv("GEMINI_BASE_URL") or ""
GEMINI_API_KEY= os.getenv("GEMINI_API_KEY") or ""
MODEL_NAME = os.getenv("MODEL_NAME") or ""

if not BASE_URL or not API_KEY or not MODEL_NAME:
    raise ValueError(
        "Please set EXAMPLE_BASE_URL, EXAMPLE_API_KEY, EXAMPLE_MODEL_NAME via env var or code."
    )

"""This example uses a custom provider for a specific agent. Steps:
1. Create a custom OpenAI client.
2. Create a `Model` that uses the custom client.
3. Set the `model` on the Agent.
"""
client = AsyncOpenAI(base_url=GEMINI_BASE_URL, api_key=GEMINI_API_KEY)

async def main():
    # This agent will use the custom LLM provider
    agent = Agent(
        name="Assistant",
        instructions="You only respond in haikus.",
        model=OpenAIChatCompletionsModel(model=MODEL_NAME, openai_client=client)
    )

    result = await Runner.run(agent, "What's the weather in Tokyo?")
    print(result.final_output)


if __name__ == "__main__":
    asyncio.run(main())
```

## Response snapshot

Successfully refactored specification based on clarifications about architecture simplification and dependency removal:

### Key Clarifications Documented

1. **Handoffs Not Needed**: Confirmed single-agent architecture sufficient for textbook chatbot - no multi-agent delegation required
2. **Tools Should Be Outcome-Focused**: Requirements should describe what agents accomplish (retrieve content, search semantically) not implementation mechanisms (function tools)
3. **LiteLLM Dependency Removed**: Gemini integration via native OpenAI Agents SDK custom client support (AsyncOpenAI with base_url/api_key)
4. **Embedding Scripts Simplified**: Batch processing doesn't require agent orchestration framework

### Changes Applied to spec.md

**User Stories**:
- User Story 3: Removed handoff references, focused on single agent retrieving textbook content
- User Story 4: Updated to use custom OpenAI client for Gemini (no LiteLLM), added environment variables (GEMINI_BASE_URL, GEMINI_API_KEY, MODEL_NAME)
- User Story 5: Removed "OpenAI Agents SDK patterns" vague reference, focused on Qdrant Cloud migration

**Functional Requirements**:
- Removed FR-007 (agent handoffs) - not needed
- Removed FR-008 (function tools implementation detail)
- Updated FR-005: Changed from LiteLLM integration to custom client configuration
- New FR-006: Outcome-focused "agents retrieve textbook content"
- New FR-007: Outcome-focused "perform vector similarity searches"
- Updated FR-008: Context maintenance (was FR-009)
- Renumbered remaining requirements (FR-009 through FR-014)

**Key Entities**:
- Simplified Agent definition (removed handoff delegation)
- Removed "Function Tool" entity (implementation detail)
- Renamed "Agent Session" to "Conversation Context" (more outcome-focused)
- Added "Custom Model Client" entity for alternative provider support

**Success Criteria**:
- SC-003: Rephrased to focus on query processing with content retrieval (removed workflow language)
- SC-006: Changed from "agent handoffs" to "conversation context maintained"
- SC-008: Updated to emphasize environment variable switching (no code changes)

**Scope & Dependencies**:
- In Scope: Changed "Configuration of Gemini model support via LiteLLM" → "via custom OpenAI client"
- In Scope: Changed "Refactoring of embedding generation scripts to use new architecture" → "to use Qdrant Cloud"
- External Dependencies: Removed entire LiteLLM section
- External Dependencies: Updated Gemini API entry noting custom client integration, no additional dependencies
- External Dependencies: Added note to OpenAI Agents SDK about custom client support

**Technical Notes**:
- Updated Agent Creation Pattern to show custom AsyncOpenAI client for Gemini
- Updated environment variables list with GEMINI_BASE_URL, GEMINI_API_KEY, MODEL_NAME

**Clarifications Section**:
- Added new section documenting all four clarification Q&A pairs for Session 2025-12-28

### Architecture Impact

- **Simplified**: Single-agent architecture (no handoff complexity)
- **Fewer Dependencies**: Removed LiteLLM entirely (reduces supply chain risk, simpler install)
- **Clearer Requirements**: Outcome-focused (testable user value) vs implementation-focused (technical mechanisms)
- **More Flexible**: Environment variable switching enables model changes without code modification

## Outcome

- ✅ Impact: Specification significantly simplified and clarified; removed unnecessary architectural complexity (handoffs) and external dependency (LiteLLM); requirements now outcome-focused and technology-agnostic
- 🧪 Tests: Requirements testability improved - each requirement describes measurable user-facing capability
- 📁 Files: Updated spec.md (added Clarifications section, refactored 8 sections: User Stories 3-5, Functional Requirements, Key Entities, Success Criteria, Scope, Dependencies, Technical Notes)
- 🔁 Next prompts: `/sp.plan` to design implementation architecture with simplified single-agent approach and direct Gemini integration
- 🧠 Reflection: Clarification process successfully identified over-engineering (handoffs for single chatbot), implementation leakage (tools in requirements), and unnecessary dependencies (LiteLLM when SDK supports custom clients); spec now cleaner and more maintainable

## Evaluation notes (flywheel)

- Failure modes observed: Initial spec included implementation details in requirements and unnecessary architectural patterns; corrected via clarification workflow
- Graders run and results (PASS/FAIL): Outcome-focused requirement validation PASS (FR-006, FR-007 now describe capabilities not mechanisms); Dependency minimization PASS (LiteLLM removed, Gemini via native SDK support)
- Prompt variant (if applicable): Standard /sp.clarify workflow with user-provided code example demonstrating simpler integration path
- Next experiment (smallest change to try): Continue validating specification quality before planning - ensure no other implementation details leaked into requirements
