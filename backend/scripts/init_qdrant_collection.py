"""
Qdrant Cloud collection initialization script.

This script creates the 'textbook_content' collection in Qdrant Cloud with
the correct configuration for 384-dimensional embeddings using COSINE similarity.

Usage:
    python backend/scripts/init_qdrant_collection.py
"""

import os
import sys
from pathlib import Path
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models

# Add parent directory to path to import backend modules
sys.path.insert(0, str(Path(__file__).parent.parent))

# Load environment variables
load_dotenv()

def init_collection():
    """Initialize the Qdrant Cloud collection"""
    print("=" * 50)
    print("Qdrant Cloud Collection Initialization")
    print("=" * 50)

    # Get environment variables
    cluster_endpoint = os.getenv("QDRANT_CLUSTER_ENDPOINT")
    api_key = os.getenv("QDRANT_API_KEY")

    if not cluster_endpoint or not api_key:
        print("✗ Error: QDRANT_CLUSTER_ENDPOINT and QDRANT_API_KEY must be set")
        print("  Check your .env file or environment variables")
        return 1

    # Connect to Qdrant Cloud
    try:
        print(f"\nConnecting to Qdrant Cloud at {cluster_endpoint}...")
        client = QdrantClient(
            url=cluster_endpoint,
            api_key=api_key,
            timeout=10.0
        )

        # Test connection
        collections = client.get_collections()
        print("✓ Connected successfully")

    except Exception as e:
        print(f"✗ Connection failed: {e}")
        return 1

    # Create collection
    collection_name = "textbook_content"
    try:
        # Check if collection already exists
        existing_collections = [col.name for col in collections.collections]
        if collection_name in existing_collections:
            print(f"\n✓ Collection '{collection_name}' already exists")

            # Optionally show collection info
            collection_info = client.get_collection(collection_name)
            print(f"  Vectors count: {collection_info.points_count}")
            print(f"  Vector size: {collection_info.config.params.vectors.size}")
            print(f"  Distance metric: {collection_info.config.params.vectors.distance}")

        else:
            print(f"\nCreating collection: {collection_name}")

            # Create collection with 384-dimensional vectors (sentence-transformers)
            # Using COSINE similarity metric
            client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=384,  # all-MiniLM-L6-v2 embedding dimension
                    distance=models.Distance.COSINE
                )
            )

            print("✓ Collection created successfully")

            # Configure HNSW index for fast approximate nearest neighbor search
            client.update_collection(
                collection_name=collection_name,
                hnsw_config=models.HnswConfigDiff(
                    m=16,  # Number of edges per node
                    ef_construct=100  # Size of the dynamic candidate list
                )
            )

            print("✓ Index configured: HNSW with ef_construct=100, m=16")

        print("✓ Ready for indexing")
        print("=" * 50)
        return 0

    except Exception as e:
        print(f"✗ Failed to create collection: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(init_collection())
