"""
Environment validation script for backend application.

This script validates that all required environment variables are set correctly
and that connections to external services (Qdrant Cloud, LLM APIs) can be established.

Usage:
    python backend/scripts/validate_env.py
    python backend/scripts/validate_env.py --qdrant-only  # Only test Qdrant
    python backend/scripts/validate_env.py --llm-only     # Only test LLM
"""

import os
import sys
from pathlib import Path
from dotenv import load_dotenv
from typing import Optional

# Add parent directory to path to import backend modules
sys.path.insert(0, str(Path(__file__).parent.parent))

# Load environment variables
load_dotenv()


def check_mark(success: bool) -> str:
    """Return checkmark or X based on success"""
    return "✓" if success else "✗"


def validate_qdrant_config() -> bool:
    """Validate Qdrant Cloud configuration and connectivity"""
    from qdrant_client import QdrantClient

    print("\n=== Qdrant Cloud Configuration ===")

    try:
        from src.config.env import get_environment_config
        config = get_environment_config()
        cluster_endpoint = config.qdrant_cluster_endpoint
        api_key = config.qdrant_api_key
        timeout = config.qdrant_timeout
        print(f"Endpoint: {cluster_endpoint}")
        print(f"Timeout: {timeout}s")
    except ValueError as e:
        print(f"Environment configuration error: {e}")
        return False
    except ImportError as e:
        # Fallback to direct env vars if config module not available
        cluster_endpoint = os.getenv("QDRANT_CLUSTER_ENDPOINT")
        api_key = os.getenv("QDRANT_API_KEY")
        timeout = float(os.getenv("QDRANT_TIMEOUT", "10.0"))
        print(f"Using direct environment variables (config module not available)")

    endpoint_ok = cluster_endpoint is not None and cluster_endpoint.startswith("https://")
    api_key_ok = api_key is not None and len(api_key) > 0

    print(f"{check_mark(endpoint_ok)} QDRANT_CLUSTER_ENDPOINT configured")
    if not endpoint_ok:
        print("  ⚠ Must be set and start with https://")

    print(f"{check_mark(api_key_ok)} QDRANT_API_KEY configured")
    if not api_key_ok:
        print("  ⚠ Must be set to a valid API key")

    # Test connection if both are configured
    if endpoint_ok and api_key_ok:
        try:
            client = QdrantClient(url=cluster_endpoint, api_key=api_key, timeout=timeout)
            collections = client.get_collections()
            print(f"{check_mark(True)} Qdrant Cloud connection successful")
            print(f"  Collections: {len(collections.collections)}")

            # Check for textbook_content collection
            collection_names = [c.name for c in collections.collections]
            if "textbook_content" in collection_names:
                print(f"  ✓ 'textbook_content' collection exists")
            else:
                print(f"  ⚠ 'textbook_content' collection not found (run index_content.py)")

            return True
        except Exception as e:
            print(f"{check_mark(False)} Qdrant Cloud connection failed: {e}")
            return False

    return False


def validate_llm_config() -> bool:
    """Validate LLM API configuration (Gemini or OpenAI)"""
    print("\n=== LLM Configuration ===")

    try:
        from src.config.env import get_environment_config
        config = get_environment_config()

        gemini_configured = config.is_gemini_configured()
        openai_configured = config.is_openai_configured()
        provider = config.get_llm_provider()
        model_name = config.model_name

        print(f"LLM Provider: {provider}")
        print(f"Model: {model_name}")

        if gemini_configured:
            print(f"Gemini Base URL: {config.gemini_base_url}")
            print(f"Gemini API Key: {'*' * 20}{config.gemini_api_key[-4:] if len(config.gemini_api_key) > 4 else '****'}")

        if openai_configured:
            print(f"OpenAI API Key: {'*' * 20}{config.openai_api_key[-4:] if len(config.openai_api_key) > 4 else '****'}")

        if not gemini_configured and not openai_configured:
            print(f"{check_mark(False)} No LLM provider configured")
            print("  ⚠ Configure either Gemini or OpenAI")
            return False

        return True

    except ValueError as e:
        print(f"Configuration error: {e}")
        return False
    except ImportError:
        # Fallback to direct env vars
        gemini_base_url = os.getenv("GEMINI_BASE_URL")
        gemini_api_key = os.getenv("GEMINI_API_KEY")
        openai_api_key = os.getenv("OPENAI_API_KEY")
        model_name = os.getenv("MODEL_NAME")

        has_gemini = gemini_base_url and gemini_api_key
        has_openai = openai_api_key is not None

        if has_gemini:
            print(f"{check_mark(True)} Gemini configuration detected")
            print(f"  Base URL: {gemini_base_url}")
        if has_openai:
            print(f"{check_mark(True)} OpenAI configuration detected")
        if not has_gemini and not has_openai:
            print(f"{check_mark(False)} No LLM provider configured")
            return False
        if model_name:
            print(f"Model: {model_name}")
        return True


def validate_embedding_config() -> bool:
    """Validate embedding model configuration"""
    print("\n=== Embedding Configuration ===")

    try:
        from src.config.env import get_environment_config
        config = get_environment_config()
        embedding_model = config.embedding_model
        print(f"Embedding model: {embedding_model}")
    except (ValueError, ImportError):
        embedding_model = os.getenv("EMBEDDING_MODEL", "all-MiniLM-L6-v2")
        print(f"Using default embedding model: {embedding_model}")

    # Try to load the model
    try:
        from sentence_transformers import SentenceTransformer
        model = SentenceTransformer(embedding_model)
        dim = model.get_sentence_embedding_dimension()
        print(f"{check_mark(True)} Embedding model loaded successfully (dim={dim})")
        return True
    except Exception as e:
        print(f"{check_mark(False)} Failed to load embedding model: {e}")
        return False


def test_qdrant_indexing() -> bool:
    """Test Qdrant indexing capability"""
    print("\n=== Qdrant Indexing Test ===")

    try:
        from src.services.qdrant_service import QdrantService
        from sentence_transformers import SentenceTransformer

        print("Initializing services...")
        qdrant_service = QdrantService()
        encoder = SentenceTransformer("all-MiniLM-L6-v2")

        print("Creating test embedding...")
        test_text = "This is a test sentence for embedding."
        embedding = encoder.encode(test_text).tolist()

        print("Uploading test point...")
        test_id = "test_point_001"
        qdrant_service.store_embeddings(
            content_id=test_id,
            embedding=embedding,
            payload={
                "title": "Test Document",
                "body": test_text,
                "source_path": "/test",
                "metadata": {"test": True}
            }
        )
        print(f"{check_mark(True)} Test point uploaded successfully")

        # Verify it exists
        results = qdrant_service.search_similar(embedding, limit=1)
        if results:
            print(f"{check_mark(True)} Test point retrieved successfully (score: {results[0]['score']:.4f})")
        else:
            print(f"{check_mark(False)} Test point not found in search")

        # Clean up test point
        try:
            qdrant_service.delete_content(test_id)
            print(f"{check_mark(True)} Test point cleaned up")
        except Exception as e:
            print(f"⚠ Cleanup failed: {e}")

        return True

    except Exception as e:
        print(f"{check_mark(False)} Indexing test failed: {e}")
        return False


def main():
    """Run all validation checks"""
    import argparse

    parser = argparse.ArgumentParser(description="Validate backend environment configuration")
    parser.add_argument('--qdrant-only', action='store_true', help='Only test Qdrant connection')
    parser.add_argument('--llm-only', action='store_true', help='Only test LLM configuration')
    parser.add_argument('--indexing', action='store_true', help='Test Qdrant indexing capability')
    args = parser.parse_args()

    print("=" * 50)
    print("Backend Environment Validation")
    print("=" * 50)
    print(f"Timestamp: {datetime.now().isoformat()}")

    # Check for required packages
    try:
        import qdrant_client
        print(f"Qdrant Client: {qdrant_client.__version__}")
    except ImportError:
        print("⚠ Qdrant client not installed")

    results = {}

    if args.qdrant_only:
        results["qdrant"] = validate_qdrant_config()
    elif args.llm_only:
        results["llm"] = validate_llm_config()
        results["embedding"] = validate_embedding_config()
    else:
        # Run all checks
        results["qdrant"] = validate_qdrant_config()
        results["llm"] = validate_llm_config()
        results["embedding"] = validate_embedding_config()

        if args.indexing:
            results["indexing"] = test_qdrant_indexing()

    print("\n" + "=" * 50)
    print("Summary")
    print("=" * 50)

    all_passed = all(results.values())
    for check, passed in results.items():
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"  {check.upper()}: {status}")

    print("=" * 50)
    if all_passed:
        print("✓ All checks passed! Ready to proceed.")
        return 0
    else:
        print("✗ Some checks failed. Please fix the issues above.")
        return 1


if __name__ == "__main__":
    from datetime import datetime
    sys.exit(main())
