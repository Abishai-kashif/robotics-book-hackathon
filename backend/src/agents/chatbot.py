"""
Textbook chatbot agent implementation using OpenAI Agents SDK.

This module defines the conversational agent for the Physical AI & Humanoid Robotics
textbook assistant, including agent instructions and query processing logic.
"""

import logging
from typing import Optional, Dict, Any
from agents import (
    Agent, Runner, AsyncOpenAI,
    set_default_openai_api,
    set_default_openai_client,
    )

from .models import get_model

logger = logging.getLogger(__name__)

# Agent instructions for textbook assistant
TEXTBOOK_AGENT_INSTRUCTIONS = """
You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.

Your role is to:
- Answer questions about robotics concepts, ROS 2, humanoid control systems, and physical AI
- Provide clear, accurate explanations based on the textbook content
- Cite specific sections or chapters when referencing textbook material
- Help students understand complex topics by breaking them down into simpler concepts
- Suggest related topics or chapters for further reading when appropriate

Guidelines:
- Always prioritize accuracy over speculation
- If you're unsure about something, say so and suggest where to find the information
- Use the provided textbook content context to inform your answers
- Keep responses concise but thorough
- Use technical terminology appropriately for a graduate-level audience
- When discussing code examples, explain both what the code does and why it matters

Context: You will receive relevant textbook excerpts to help answer user queries.
Use this context to provide informed, accurate responses.
"""


def create_textbook_agent(name: str = "Textbook Assistant") -> Agent:
    """
    Create and configure the textbook chatbot agent.

    This function initializes an Agent instance with appropriate instructions
    and model configuration for assisting with textbook questions.

    Args:
        name: Display name for the agent (default: "Textbook Assistant")

    Returns:
        Configured Agent instance ready to process queries

    Raises:
        ValueError: If model configuration is invalid
    """
    logger.info(f"Creating textbook agent: {name}")

    # Get configured model (Gemini or OpenAI based on environment)
    model = get_model()

    # Create agent with instructions and model
    agent = Agent(
        name=name,
        instructions=TEXTBOOK_AGENT_INSTRUCTIONS,
        model=model
    )

    logger.info(f"Agent '{name}' created successfully")
    return agent


async def process_query_with_agent(
    query: str,
    context: Optional[str] = None,
    agent: Optional[Agent] = None
) -> Dict[str, Any]:
    """
    Process a user query using the textbook agent.

    This function takes a user query, optionally enriched with textbook context,
    and processes it through the OpenAI Agents SDK using Runner.run().

    Args:
        query: User's question or prompt
        context: Optional textbook content context to include
        agent: Optional pre-created agent instance (creates new if None)

    Returns:
        Dictionary containing:
            - answer: Agent's response text
            - success: Boolean indicating successful processing
            - error: Error message if processing failed

    Example:
        >>> result = await process_query_with_agent(
        ...     query="What is ROS 2?",
        ...     context="ROS 2 is a flexible framework..."
        ... )
        >>> print(result["answer"])
    """
    try:
        # Create agent if not provided
        if agent is None:
            agent = create_textbook_agent()

        # Build the full prompt with context
        if context:
            full_prompt = f"""Context from textbook:
{context}

User question: {query}

Please answer based on the provided context and your knowledge of robotics and AI."""
        else:
            full_prompt = query

        logger.info(f"Processing query: {query[:100]}...")

        # Run the agent using Runner
        result = await Runner.run(agent, full_prompt)

        # Extract response from result
        # The Runner returns a RunResult object with final_output
        if result and result.final_output is not None:
            # final_output contains the agent's response
            answer = result.final_output
            if hasattr(answer, 'content'):
                answer = answer.content

            logger.info("Query processed successfully")
            return {
                "answer": str(answer),
                "success": True,
                "error": None
            }
        else:
            logger.warning("Agent returned empty result")
            return {
                "answer": "",
                "success": False,
                "error": "Agent returned empty response"
            }

    except Exception as e:
        # Enhanced error handling for API failures
        error_msg = str(e)
        logger.error(f"Error processing query with agent: {e}")

        # Check for common Gemini/OpenAI API errors
        if "401" in error_msg or "authentication" in error_msg.lower():
            return {
                "answer": "",
                "success": False,
                "error": "Authentication failed. Please check your API key configuration (GEMINI_API_KEY or OPENAI_API_KEY)."
            }
        elif "429" in error_msg or "rate limit" in error_msg.lower():
            return {
                "answer": "",
                "success": False,
                "error": "API rate limit exceeded. Please try again in a few moments."
            }
        elif "timeout" in error_msg.lower():
            return {
                "answer": "",
                "success": False,
                "error": "Request timeout. The LLM service took too long to respond. Please try again."
            }
        else:
            return {
                "answer": "",
                "success": False,
                "error": f"Error processing query: {error_msg}"
            }


# Global agent instance (singleton for reuse)
_global_agent: Optional[Agent] = None


def get_global_agent() -> Agent:
    """
    Get or create the global textbook agent instance.

    This function maintains a singleton agent instance to avoid
    recreating the agent for every query.

    Returns:
        Global Agent instance
    """
    global _global_agent

    if _global_agent is None:
        _global_agent = create_textbook_agent()

    return _global_agent


def reset_global_agent() -> None:
    """
    Reset the global agent instance.

    Useful for testing or when configuration changes require agent recreation.
    """
    global _global_agent
    _global_agent = None
    logger.info("Global agent reset")
