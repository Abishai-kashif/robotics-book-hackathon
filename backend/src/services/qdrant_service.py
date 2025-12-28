"""
Qdrant service for vector database operations using Qdrant Cloud
"""
from qdrant_client.http import models
from typing import List, Dict, Any
import logging

from ..database.qdrant_client import get_qdrant_client, connect_with_retry

logger = logging.getLogger(__name__)

class QdrantService:
    def __init__(self, use_retry: bool = True, max_retries: int = 3):
        """
        Initialize Qdrant service with cloud-only configuration.

        Args:
            use_retry: Whether to use retry logic for connection (default: True)
            max_retries: Maximum number of connection retry attempts (default: 3)
        """
        self.collection_name = "textbook_content"
        self.use_retry = use_retry
        self.max_retries = max_retries
        self._client = None  # Initialize client lazily

    @property
    def client(self):
        """Lazily initialize the Qdrant client to avoid multiprocessing issues"""
        if self._client is None:
            # Use retry logic if enabled
            if self.use_retry:
                self._client = connect_with_retry(max_retries=self.max_retries)
            else:
                self._client = get_qdrant_client()

            # Initialize the collection after client is ready
            self._initialize_collection()

        return self._client

    def _initialize_collection(self):
        """Initialize the collection for storing textbook content embeddings"""
        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
            logger.info(f"Collection {self.collection_name} already exists")
        except:
            # Create collection if it doesn't exist
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=384, distance=models.Distance.COSINE),
            )
            logger.info(f"Created collection {self.collection_name}")

    def store_embeddings(self, content_id: str, embedding: List[float], payload: Dict[str, Any]):
        """Store embeddings in Qdrant"""
        try:
            self.client.upsert(
                collection_name=self.collection_name,
                points=[
                    models.PointStruct(
                        id=content_id,
                        vector=embedding,
                        payload=payload
                    )
                ]
            )
            logger.info(f"Stored embedding for content_id: {content_id}")
        except Exception as e:
            logger.error(f"Error storing embedding: {e}")
            raise

    def search_similar(self, query_embedding: List[float], limit: int = 5) -> List[Dict[str, Any]]:
        """Search for similar content based on embedding"""
        try:
            results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=limit
            )

            return [
                {
                    "content_id": point.id,
                    "payload": point.payload,
                    "score": point.score
                }
                for point in results.points
            ]
        except Exception as e:
            logger.error(f"Error searching similar content: {e}")
            raise

    def delete_content(self, content_id: str):
        """Delete content by ID"""
        try:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=[content_id]
                )
            )
            logger.info(f"Deleted content with ID: {content_id}")
        except Exception as e:
            logger.error(f"Error deleting content: {e}")
            raise

    def get_all_content_ids(self) -> List[str]:
        """Get all content IDs in the collection"""
        try:
            records, _ = self.client.scroll(
                collection_name=self.collection_name,
                limit=10000  # Adjust as needed
            )
            return [record.id for record in records]
        except Exception as e:
            logger.error(f"Error getting content IDs: {e}")
            raise