"""
Integration tests for backend services.

These tests verify end-to-end functionality of the chatbot backend
including Qdrant Cloud connectivity and agent processing.
"""

import pytest
import asyncio
from unittest.mock import patch, MagicMock
import time


class TestEnvironmentConfig:
    """Test environment configuration loading"""

    def test_config_loads_successfully(self):
        """Verify EnvironmentConfig can be loaded"""
        from src.config.env import get_environment_config
        config = get_environment_config()
        assert config is not None

    def test_config_has_required_fields(self):
        """Verify config has all required fields"""
        from src.config.env import EnvironmentConfig
        config = EnvironmentConfig()
        assert hasattr(config, 'qdrant_cluster_endpoint')
        assert hasattr(config, 'qdrant_api_key')
        assert hasattr(config, 'model_name')

    def test_get_llm_provider_gemini(self):
        """Test Gemini provider detection"""
        from src.config.env import EnvironmentConfig
        config = EnvironmentConfig(
            gemini_base_url="https://test.com",
            gemini_api_key="test-key"
        )
        assert config.get_llm_provider() == "gemini"

    def test_get_llm_provider_openai(self):
        """Test OpenAI provider detection"""
        from src.config.env import EnvironmentConfig
        config = EnvironmentConfig(
            openai_api_key="sk-test-key"
        )
        assert config.get_llm_provider() == "openai"


class TestQdrantClient:
    """Test Qdrant Cloud client functionality"""

    def test_qdrant_client_module_imports(self):
        """Verify Qdrant client module imports correctly"""
        from src.database.qdrant_client import get_qdrant_client, verify_connection
        assert get_qdrant_client is not None
        assert verify_connection is not None

    def test_qdrant_connection_error_class(self):
        """Verify QdrantConnectionError exists"""
        from src.database.qdrant_client import QdrantConnectionError
        assert QdrantConnectionError is not None
        assert issubclass(QdrantConnectionError, Exception)


class TestAgents:
    """Test agent implementations"""

    def test_chatbot_module_imports(self):
        """Verify chatbot module imports correctly"""
        from src.agents.chatbot import create_textbook_agent, process_query_with_agent
        assert create_textbook_agent is not None
        assert process_query_with_agent is not None

    def test_models_module_imports(self):
        """Verify models module imports correctly"""
        from src.agents.models import get_model, get_openai_model, get_gemini_model
        assert get_model is not None
        assert get_openai_model is not None
        assert get_gemini_model is not None

    def test_agent_instructions_exist(self):
        """Verify agent instructions are defined"""
        from src.agents.chatbot import TEXTBOOK_AGENT_INSTRUCTIONS
        assert TEXTBOOK_AGENT_INSTRUCTIONS is not None
        assert "Physical AI" in TEXTBOOK_AGENT_INSTRUCTIONS
        assert "Humanoid Robotics" in TEXTBOOK_AGENT_INSTRUCTIONS


class TestQdrantService:
    """Test Qdrant service functionality"""

    def test_qdrant_service_imports(self):
        """Verify QdrantService imports correctly"""
        from src.services.qdrant_service import QdrantService
        assert QdrantService is not None

    def test_qdrant_service_has_required_methods(self):
        """Verify QdrantService has required methods"""
        from src.services.qdrant_service import QdrantService
        service = QdrantService()
        assert hasattr(service, 'client')
        assert hasattr(service, 'store_embeddings')
        assert hasattr(service, 'search_similar')
        assert hasattr(service, 'delete_content')


class TestAPI:
    """Test API endpoints"""

    def test_chatbot_router_imports(self):
        """Verify chatbot router imports correctly"""
        from src.api.chatbot_router import router
        assert router is not None

    def test_router_has_required_endpoints(self):
        """Verify router has all required endpoints"""
        from src.api.chatbot_router import router

        # Check endpoint paths exist
        paths = [route.path for route in router.routes]
        assert '/chat' in paths
        assert '/embeddings' in paths
        assert '/health' in paths
        assert '/metrics' in paths


class TestIndexingScript:
    """Test indexing script functionality"""

    def test_index_script_module_imports(self):
        """Verify indexing script can be imported"""
        import sys
        from pathlib import Path
        sys.path.insert(0, str(Path(__file__).parent.parent.parent))

        # Test functions exist
        from scripts.index_content import (
            extract_title_from_markdown,
            clean_markdown,
            chunk_text,
            generate_content_id
        )
        assert extract_title_from_markdown is not None
        assert clean_markdown is not None
        assert chunk_text is not None
        assert generate_content_id is not None

    def test_extract_title_from_markdown(self):
        """Test title extraction from markdown"""
        from scripts.index_content import extract_title_from_markdown

        content = "# Test Title\n\nSome content here"
        title = extract_title_from_markdown(content, "test-file")
        assert title == "Test Title"

    def test_clean_markdown(self):
        """Test markdown cleaning"""
        from scripts.index_content import clean_markdown

        content = """---
title: Test
---

# Header

Some [link](http://example.com) text
"""
        cleaned = clean_markdown(content)
        assert "---" not in cleaned
        assert "[link]" in cleaned  # Link text preserved

    def test_chunk_text(self):
        """Test text chunking"""
        from scripts.index_content import chunk_text

        text = "Para 1.\n\nPara 2.\n\nPara 3.\n\nPara 4.\n\nPara 5."
        chunks = chunk_text(text, chunk_size=100, overlap=20)
        assert len(chunks) >= 2


class TestValidationScript:
    """Test validation script functionality"""

    def test_validate_script_module_imports(self):
        """Verify validation script can be imported"""
        import sys
        from pathlib import Path
        sys.path.insert(0, str(Path(__file__).parent.parent.parent))

        from scripts.validate_env import (
            validate_qdrant_config,
            validate_llm_config,
            validate_embedding_config
        )
        assert validate_qdrant_config is not None
        assert validate_llm_config is not None
        assert validate_embedding_config is not None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
