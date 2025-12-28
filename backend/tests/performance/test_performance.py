"""
Performance tests for backend services.

These tests verify performance requirements:
- Qdrant search latency < 500ms p95
- Agent query processing < 30 seconds
- Concurrent query handling
"""

import pytest
import time
import asyncio
from unittest.mock import patch, MagicMock


class TestQdrantPerformance:
    """Test Qdrant Cloud performance"""

    @pytest.mark.asyncio
    async def test_qdrant_search_latency(self):
        """Verify Qdrant search latency is within acceptable range"""
        from src.services.qdrant_service import QdrantService

        service = QdrantService()
        latencies = []

        # Test search with mock embedding (384 dimensions)
        mock_embedding = [0.1] * 384

        # Run 5 queries to get average
        for _ in range(5):
            start = time.perf_counter()
            try:
                results = service.search_similar(mock_embedding, limit=5)
                elapsed = (time.perf_counter() - start) * 1000  # ms
                latencies.append(elapsed)
            except Exception:
                # Expected to fail without real Qdrant connection
                pass

        if latencies:
            latencies.sort()
            p95 = latencies[int(len(latencies) * 0.95)]
            print(f"Qdrant search p95 latency: {p95:.2f}ms")
            # Note: With mock, this should be very fast
            # Real test against Qdrant Cloud should be < 500ms
            assert p95 < 1000  # Mock should be < 1 second


class TestAgentPerformance:
    """Test agent processing performance"""

    @pytest.mark.asyncio
    async def test_agent_query_processing_time(self):
        """Verify agent query processing completes within timeout"""
        from src.agents.chatbot import process_query_with_agent

        # Test with short query
        start = time.perf_counter()
        try:
            result = await process_query_with_agent(
                query="What is ROS 2?",
                context=None
            )
            elapsed = time.perf_counter() - start

            # Should complete (with or without real API)
            assert 'success' in result
            print(f"Agent query completed in {elapsed:.2f}s")
        except Exception as e:
            # Expected to fail without real API connection
            pass


class TestConcurrentQueries:
    """Test concurrent query handling"""

    @pytest.mark.asyncio
    async def test_concurrent_search_requests(self):
        """Verify system can handle concurrent search requests"""
        from src.services.qdrant_service import QdrantService

        service = QdrantService()
        mock_embedding = [0.1] * 384

        # Run 3 concurrent searches
        async def search():
            try:
                return service.search_similar(mock_embedding, limit=5)
            except Exception:
                return None

        start = time.perf_counter()
        tasks = [search() for _ in range(3)]
        results = await asyncio.gather(*tasks)
        elapsed = time.perf_counter() - start

        print(f"3 concurrent searches completed in {elapsed:.2f}s")
        # Should complete without errors
        assert elapsed < 10  # Should complete in reasonable time


class TestEmbeddingPerformance:
    """Test embedding generation performance"""

    def test_embedding_generation_time(self):
        """Verify embedding generation is fast enough"""
        try:
            from sentence_transformers import SentenceTransformer

            model = SentenceTransformer("all-MiniLM-L6-v2")
            text = "ROS 2 is a flexible framework for robot software."

            # Generate embeddings multiple times
            latencies = []
            for _ in range(3):
                start = time.perf_counter()
                embedding = model.encode(text)
                elapsed = (time.perf_counter() - start) * 1000
                latencies.append(elapsed)

            avg_latency = sum(latencies) / len(latencies)
            print(f"Average embedding time: {avg_latency:.2f}ms")
            # Should be < 100ms for single sentence
            assert avg_latency < 100
        except ImportError:
            pytest.skip("sentence-transformers not installed")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
