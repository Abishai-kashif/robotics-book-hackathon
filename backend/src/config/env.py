"""
Environment configuration module with validation.

This module provides centralized environment variable management with
fail-fast validation for all required credentials and settings.
"""

import os
from typing import Optional
from dotenv import load_dotenv
from pydantic import BaseModel, Field, field_validator, model_validator

# Load environment variables from .env file
load_dotenv()


class EnvironmentConfig(BaseModel):
    """
    Environment configuration with validation for cloud services.

    This class loads and validates all required environment variables,
    ensuring the application fails fast if configuration is incomplete or invalid.
    """

    # Qdrant Cloud Configuration (REQUIRED)
    qdrant_cluster_endpoint: str = Field(
        default_factory=lambda: os.getenv("QDRANT_CLUSTER_ENDPOINT", ""),
        description="Full URL to Qdrant Cloud cluster (must start with https://)"
    )
    qdrant_api_key: str = Field(
        default_factory=lambda: os.getenv("QDRANT_API_KEY", ""),
        description="Authentication key for Qdrant Cloud"
    )
    qdrant_timeout: float = Field(
        default_factory=lambda: float(os.getenv("QDRANT_TIMEOUT", "10.0")),
        description="Connection timeout in seconds"
    )

    # LLM Provider Configuration (at least one required)
    gemini_base_url: Optional[str] = Field(
        default_factory=lambda: os.getenv("GEMINI_BASE_URL"),
        description="Gemini API endpoint for custom client"
    )
    gemini_api_key: Optional[str] = Field(
        default_factory=lambda: os.getenv("GEMINI_API_KEY"),
        description="Gemini authentication key"
    )
    openai_api_key: Optional[str] = Field(
        default_factory=lambda: os.getenv("OPENAI_API_KEY"),
        description="OpenAI authentication key"
    )
    model_name: str = Field(
        default_factory=lambda: os.getenv("MODEL_NAME", "gemini-2.0-flash"),
        description="LLM model identifier"
    )

    # Embedding Model Configuration
    embedding_model: str = Field(
        default_factory=lambda: os.getenv("EMBEDDING_MODEL", "all-MiniLM-L6-v2"),
        description="sentence-transformers model name"
    )

    # Application Settings
    python_version: str = Field(
        default_factory=lambda: os.getenv("PYTHON_VERSION", "3.10"),
        description="Python runtime version"
    )

    class Config:
        """Pydantic configuration"""
        frozen = True  # Make configuration immutable after creation
        validate_assignment = True

    @field_validator("qdrant_cluster_endpoint")
    @classmethod
    def validate_qdrant_endpoint(cls, v: str) -> str:
        """Validate Qdrant cluster endpoint format"""
        if not v:
            raise ValueError("QDRANT_CLUSTER_ENDPOINT environment variable is required")
        if not v.startswith("https://"):
            raise ValueError(
                f"QDRANT_CLUSTER_ENDPOINT must start with https:// (got: {v}). "
                "Cloud connections require TLS encryption."
            )
        return v

    @field_validator("qdrant_api_key")
    @classmethod
    def validate_qdrant_api_key(cls, v: str) -> str:
        """Validate Qdrant API key is non-empty"""
        if not v or not v.strip():
            raise ValueError("QDRANT_API_KEY environment variable is required")
        return v

    @model_validator(mode="after")
    def validate_llm_provider(self):
        """Validate that at least one LLM provider is configured"""
        has_gemini = self.gemini_base_url and self.gemini_api_key
        has_openai = self.openai_api_key

        if not has_gemini and not has_openai:
            raise ValueError(
                "At least one LLM provider must be configured:\n"
                "  - Gemini: Set both GEMINI_BASE_URL and GEMINI_API_KEY\n"
                "  - OpenAI: Set OPENAI_API_KEY"
            )

        return self

    @model_validator(mode="after")
    def validate_model_name(self):
        """Validate MODEL_NAME is set"""
        if not self.model_name or not self.model_name.strip():
            raise ValueError(
                "MODEL_NAME environment variable is required\n"
                "  Examples: gemini-2.5-flash-preview, gpt-4-turbo"
            )
        return self

    def get_llm_provider(self) -> str:
        """
        Determine which LLM provider to use based on configuration.

        Returns:
            "gemini" if Gemini is configured, "openai" otherwise
        """
        if self.gemini_base_url and self.gemini_api_key:
            return "gemini"
        return "openai"

    def is_gemini_configured(self) -> bool:
        """Check if Gemini provider is fully configured"""
        return bool(self.gemini_base_url and self.gemini_api_key)

    def is_openai_configured(self) -> bool:
        """Check if OpenAI provider is configured"""
        return bool(self.openai_api_key)


# Global configuration instance (loaded once at module import)
_config_instance: Optional[EnvironmentConfig] = None


def get_environment_config() -> EnvironmentConfig:
    """
    Get the global environment configuration instance.

    Configuration is loaded once on first call and cached for subsequent calls.
    This ensures consistent configuration throughout the application lifecycle.

    Returns:
        EnvironmentConfig instance with validated settings

    Raises:
        ValueError: If required environment variables are missing or invalid
    """
    global _config_instance

    if _config_instance is None:
        _config_instance = EnvironmentConfig()

    return _config_instance


def reload_environment_config() -> EnvironmentConfig:
    """
    Force reload of environment configuration (useful for testing).

    Returns:
        Newly created EnvironmentConfig instance
    """
    global _config_instance
    load_dotenv(override=True)
    _config_instance = EnvironmentConfig()
    return _config_instance
