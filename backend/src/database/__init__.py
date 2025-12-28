"""
Database module for vector database operations.

This module provides Qdrant Cloud client management and vector search services.
"""

from .qdrant_client import (
    get_qdrant_client,
    verify_connection,
    connect_with_retry,
    reset_client,
    QdrantConnectionError
)

__all__ = [
    "get_qdrant_client",
    "verify_connection",
    "connect_with_retry",
    "reset_client",
    "QdrantConnectionError"
]
