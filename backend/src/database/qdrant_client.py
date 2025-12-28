"""
Qdrant Cloud client factory with health checks and retry logic.

This module provides a centralized way to create and manage Qdrant Cloud connections
with proper error handling, timeout configuration, and connection health verification.
"""

import logging
import time
from typing import Optional
from qdrant_client import QdrantClient
from qdrant_client.http.exceptions import UnexpectedResponse

from ..config.env import get_environment_config

logger = logging.getLogger(__name__)

# Global client instance (singleton)
_qdrant_client: Optional[QdrantClient] = None


class QdrantConnectionError(Exception):
    """Raised when Qdrant Cloud connection fails"""
    pass


def get_qdrant_client(force_reconnect: bool = False) -> QdrantClient:
    """
    Get or create Qdrant Cloud client instance.

    This function returns a singleton QdrantClient connected to Qdrant Cloud
    using environment configuration. The client is cached after first creation.

    Args:
        force_reconnect: If True, force creation of new client instance

    Returns:
        QdrantClient instance connected to Qdrant Cloud

    Raises:
        QdrantConnectionError: If connection to Qdrant Cloud fails
        ValueError: If required environment variables are missing
    """
    global _qdrant_client

    if _qdrant_client is not None and not force_reconnect:
        return _qdrant_client

    # Load environment configuration
    config = get_environment_config()

    try:
        logger.info(f"Connecting to Qdrant Cloud at {config.qdrant_cluster_endpoint}")

        # Create client with cloud configuration
        client = QdrantClient(
            url=config.qdrant_cluster_endpoint,
            api_key=config.qdrant_api_key,
            timeout=config.qdrant_timeout
        )

        # Verify connection with health check
        if not verify_connection(client):
            raise QdrantConnectionError("Health check failed after connection")

        logger.info("Successfully connected to Qdrant Cloud")
        _qdrant_client = client
        return client

    except UnexpectedResponse as e:
        if e.status_code == 401:
            logger.error("Authentication failed: Invalid QDRANT_API_KEY")
            raise QdrantConnectionError(
                "Authentication failed. Check QDRANT_API_KEY environment variable."
            ) from e
        else:
            logger.error(f"Qdrant Cloud API error (status {e.status_code}): {e}")
            raise QdrantConnectionError(f"API error: {e}") from e

    except Exception as e:
        logger.error(f"Failed to connect to Qdrant Cloud: {e}")
        raise QdrantConnectionError(f"Connection failed: {e}") from e


def verify_connection(client: Optional[QdrantClient] = None) -> bool:
    """
    Verify Qdrant Cloud connection is healthy.

    This performs a lightweight operation (get_collections) to confirm
    the client can communicate with Qdrant Cloud.

    Args:
        client: QdrantClient instance to check. If None, uses global instance.

    Returns:
        True if connection is healthy, False otherwise
    """
    if client is None:
        client = _qdrant_client

    if client is None:
        logger.warning("No Qdrant client available for health check")
        return False

    try:
        # Perform lightweight operation to test connectivity
        client.get_collections()
        return True
    except Exception as e:
        logger.error(f"Qdrant health check failed: {e}")
        return False


def connect_with_retry(max_retries: int = 3, initial_delay: float = 1.0) -> QdrantClient:
    """
    Connect to Qdrant Cloud with exponential backoff retry logic.

    This function attempts to establish a connection with automatic retries
    and exponential backoff to handle transient network issues.

    Args:
        max_retries: Maximum number of retry attempts (default: 3)
        initial_delay: Initial delay between retries in seconds (default: 1.0)

    Returns:
        QdrantClient instance connected to Qdrant Cloud

    Raises:
        QdrantConnectionError: If all retry attempts fail
    """
    delay = initial_delay

    for attempt in range(1, max_retries + 1):
        try:
            logger.info(f"Connection attempt {attempt}/{max_retries}")
            client = get_qdrant_client(force_reconnect=True)
            logger.info("Connection successful")
            return client

        except QdrantConnectionError as e:
            if attempt == max_retries:
                logger.error(f"All {max_retries} connection attempts failed")
                raise

            logger.warning(f"Attempt {attempt} failed: {e}. Retrying in {delay}s...")
            time.sleep(delay)
            delay *= 2  # Exponential backoff

    # Should never reach here, but just in case
    raise QdrantConnectionError(f"Failed after {max_retries} attempts")


def reset_client() -> None:
    """
    Reset the global Qdrant client instance.

    This is useful for testing or when configuration changes require reconnection.
    """
    global _qdrant_client
    _qdrant_client = None
    logger.info("Qdrant client reset")
