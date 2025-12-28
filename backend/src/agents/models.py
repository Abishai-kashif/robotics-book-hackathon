"""
Model factory for OpenAI and Gemini agents.

This module provides factory functions for creating agent models with different
LLM providers (OpenAI, Gemini) using custom client configurations.
"""

import logging
from openai import AsyncOpenAI
from agents import OpenAIChatCompletionsModel

from ..config.env import get_environment_config

logger = logging.getLogger(__name__)


def get_openai_model() -> OpenAIChatCompletionsModel:
    """
    Create an OpenAI model for agent use.

    This function creates a model instance configured with OpenAI's API
    using credentials from environment variables.

    Returns:
        OpenAIChatCompletionsModel configured for OpenAI

    Raises:
        ValueError: If OPENAI_API_KEY is not configured
    """
    config = get_environment_config()

    if not config.is_openai_configured():
        raise ValueError(
            "OpenAI model requested but OPENAI_API_KEY not configured. "
            "Set OPENAI_API_KEY environment variable."
        )

    logger.info(f"Creating OpenAI model: {config.model_name}")

    # Create OpenAI client (uses OPENAI_API_KEY from environment)
    client = AsyncOpenAI(api_key=config.openai_api_key)

    # Return model configured with OpenAI client
    return OpenAIChatCompletionsModel(
        model=config.model_name,
        openai_client=client
    )


def get_gemini_model() -> OpenAIChatCompletionsModel:
    """
    Create a Gemini model for agent use via custom OpenAI client.

    This function creates a model instance that uses Gemini's API through
    a custom OpenAI-compatible client configuration.

    Returns:
        OpenAIChatCompletionsModel configured for Gemini

    Raises:
        ValueError: If Gemini configuration is incomplete
    """
    config = get_environment_config()

    if not config.is_gemini_configured():
        raise ValueError(
            "Gemini model requested but configuration incomplete. "
            "Set both GEMINI_BASE_URL and GEMINI_API_KEY environment variables."
        )

    logger.info(f"Creating Gemini model: {config.model_name}")

    # Create custom OpenAI client pointing to Gemini API
    gemini_client = AsyncOpenAI(
        base_url=config.gemini_base_url,
        api_key=config.gemini_api_key
    )

    # Return model configured with Gemini client
    return OpenAIChatCompletionsModel(
        model=config.model_name,
        openai_client=gemini_client
    )


def get_model() -> OpenAIChatCompletionsModel:
    """
    Get the configured model based on environment settings.

    This function automatically selects the appropriate model provider
    (Gemini or OpenAI) based on environment configuration.

    Returns:
        OpenAIChatCompletionsModel configured for the active provider

    Raises:
        ValueError: If no LLM provider is configured
    """
    config = get_environment_config()
    provider = config.get_llm_provider()

    logger.info(f"Selecting model provider: {provider}")

    if provider == "gemini":
        return get_gemini_model()
    elif provider == "openai":
        return get_openai_model()
    else:
        raise ValueError(
            f"Unknown LLM provider: {provider}. "
            "Configure either Gemini (GEMINI_BASE_URL + GEMINI_API_KEY) "
            "or OpenAI (OPENAI_API_KEY)."
        )
