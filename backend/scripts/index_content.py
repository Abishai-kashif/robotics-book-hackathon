"""
Script to index textbook content into Qdrant Cloud.

This script processes markdown files from the docs folder and indexes them
into the vector database with embeddings.

Usage:
    python backend/scripts/index_content.py --source book-source/docs
    python backend/scripts/index_content.py --source /path/to/docs --batch-size 32 --chunk-size 1000 --chunk-overlap 200

Arguments:
    --source: Path to documentation source directory (required)
    --batch-size: Number of embeddings to upload in a batch (default: 32)
    --chunk-size: Maximum characters per chunk (default: 1000)
    --chunk-overlap: Overlap between chunks in characters (default: 200)
    --quiet: Suppress progress output
"""
import os
import sys
import re
import hashlib
import time
import argparse
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Tuple, Optional

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from sentence_transformers import SentenceTransformer
from src.services.qdrant_service import QdrantService
from src.config.env import get_environment_config


def extract_title_from_markdown(content: str, filename: str) -> str:
    """Extract the title from markdown content."""
    # Try to find a h1 header
    match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
    if match:
        return match.group(1).strip()
    # Fallback to filename
    return filename.replace('-', ' ').replace('_', ' ').title().replace('.Md', '')


def extract_metadata_from_frontmatter(content: str) -> dict:
    """Extract metadata from markdown frontmatter."""
    metadata = {}
    frontmatter_match = re.match(r'^---\s*\n(.*?)\n---\s*\n', content, re.DOTALL)
    if frontmatter_match:
        frontmatter = frontmatter_match.group(1)
        for line in frontmatter.split('\n'):
            if ':' in line:
                key, value = line.split(':', 1)
                metadata[key.strip()] = value.strip()
    return metadata


def clean_markdown(content: str) -> str:
    """Remove frontmatter and clean markdown for indexing."""
    # Remove frontmatter
    content = re.sub(r'^---\s*\n.*?\n---\s*\n', '', content, flags=re.DOTALL)
    # Remove code blocks (keep simple code references)
    content = re.sub(r'```[\s\S]*?```', '[code block]', content)
    # Remove images
    content = re.sub(r'!\[.*?\]\(.*?\)', '', content)
    # Simplify links - keep text
    content = re.sub(r'\[([^\]]+)\]\([^\)]+\)', r'\1', content)
    # Remove HTML tags
    content = re.sub(r'<[^>]+>', '', content)
    # Clean up whitespace
    content = re.sub(r'\n{3,}', '\n\n', content)
    return content.strip()


def chunk_text(text: str, chunk_size: int = 1000, overlap: int = 200) -> list:
    """Split text into overlapping chunks."""
    chunks = []

    # Split by paragraphs first
    paragraphs = text.split('\n\n')

    current_chunk = ""
    for para in paragraphs:
        if len(current_chunk) + len(para) < chunk_size:
            current_chunk += para + "\n\n"
        else:
            if current_chunk:
                chunks.append(current_chunk.strip())
            # Start new chunk with overlap from previous
            if chunks and overlap > 0:
                # Get last part of previous chunk for overlap
                prev_text = chunks[-1]
                overlap_text = prev_text[-overlap:] if len(prev_text) > overlap else prev_text
                current_chunk = overlap_text + "\n\n" + para + "\n\n"
            else:
                current_chunk = para + "\n\n"

    # Don't forget the last chunk
    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    return chunks


def generate_content_id(source_path: str, chunk_index: int) -> str:
    """Generate a unique content ID."""
    unique_string = f"{source_path}_{chunk_index}"
    return hashlib.md5(unique_string.encode()).hexdigest()


def index_documents(
    source_path: Path,
    batch_size: int = 32,
    chunk_size: int = 1000,
    chunk_overlap: int = 200,
    quiet: bool = False
) -> Dict[str, int]:
    """
    Main function to index all documents.

    Returns a dict with stats: total, successful, failed, duration, rate
    """
    start_time = time.time()

    # Validate environment first
    if not quiet:
        print("=" * 60)
        print("Textbook Content Indexer - Qdrant Cloud")
        print("=" * 60)

    try:
        config = get_environment_config()
        if not quiet:
            print(f"\nQdrant Cloud: {config.qdrant_cluster_endpoint}")
            print(f"Model: {config.embedding_model}")
    except ValueError as e:
        print(f"ERROR: Environment configuration failed: {e}")
        raise

    # Initialize embedding model
    if not quiet:
        print("\nInitializing embedding model...")
    encoder = SentenceTransformer(config.embedding_model)
    embedding_dim = encoder.get_sentence_embedding_dimension()
    if not quiet:
        print(f"Loaded model: {config.embedding_model} (dim={embedding_dim})")

    # Initialize Qdrant service
    if not quiet:
        print("\nInitializing Qdrant service...")
    qdrant_service = QdrantService()
    # Force client initialization
    _ = qdrant_service.client
    if not quiet:
        print("Qdrant Cloud connection ready")

    # Find all markdown files
    if not source_path.exists():
        print(f"ERROR: Docs path not found: {source_path}")
        raise FileNotFoundError(f"Source path not found: {source_path}")

    md_files = list(source_path.glob("**/*.md"))
    if not quiet:
        print(f"\nFound {len(md_files)} markdown files to index")

    # Skip template files
    md_files = [f for f in md_files if 'template' not in str(f).lower()]
    if not quiet:
        print(f"After filtering templates: {len(md_files)} files")

    # Stats tracking
    stats = {
        "total_files": len(md_files),
        "successful_files": 0,
        "failed_files": 0,
        "total_chunks": 0,
        "successful_chunks": 0,
        "failed_chunks": 0,
        "errors": []
    }

    # Batch processing for efficiency
    batch_points = []
    batch_count = 0

    def upload_batch():
        """Upload accumulated batch points to Qdrant"""
        nonlocal batch_count
        if batch_points:
            try:
                qdrant_service.client.upsert(
                    collection_name=qdrant_service.collection_name,
                    points=batch_points
                )
                stats["successful_chunks"] += len(batch_points)
            except Exception as e:
                stats["failed_chunks"] += len(batch_points)
                stats["errors"].append(f"Batch upload failed: {e}")
            finally:
                batch_points.clear()
                batch_count += 1

    for file_idx, file_path in enumerate(md_files, 1):
        try:
            # Progress output every 10 documents
            if not quiet and file_idx % 10 == 0:
                elapsed = time.time() - start_time
                rate = file_idx / elapsed if elapsed > 0 else 0
                print(f"  Progress: {file_idx}/{len(md_files)} files ({rate:.1f} files/s)")

            # Read file content
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            if len(content.strip()) < 100:
                if not quiet:
                    print(f"  Skipping (too short): {file_path.name}")
                continue

            # Extract metadata
            title = extract_title_from_markdown(content, file_path.stem)
            metadata = extract_metadata_from_frontmatter(content)

            # Clean and chunk the content
            cleaned_content = clean_markdown(content)
            chunks = chunk_text(cleaned_content, chunk_size=chunk_size, overlap=chunk_overlap)

            if not chunks:
                if not quiet:
                    print(f"  Skipping (no content after cleaning): {file_path.name}")
                continue

            # Calculate relative path for source_path
            relative_path = file_path.relative_to(source_path)
            doc_source_path = f"/docs/{str(relative_path).replace(os.sep, '/')}"

            if not quiet:
                print(f"\n[{file_idx}/{len(md_files)}] Indexing: {title}")
                print(f"  Source: {doc_source_path}")
                print(f"  Chunks: {len(chunks)}")

            # Process each chunk
            for i, chunk in enumerate(chunks):
                content_id = generate_content_id(doc_source_path, i)

                # Generate embedding
                embedding = encoder.encode(chunk).tolist()

                # Prepare payload
                payload = {
                    "title": title,
                    "body": chunk,
                    "source_path": doc_source_path,
                    "metadata": {
                        **metadata,
                        "chunk_index": i,
                        "total_chunks": len(chunks),
                        "filename": file_path.name
                    }
                }

                # Add to batch
                point = {
                    "id": content_id,
                    "vector": embedding,
                    "payload": payload
                }
                batch_points.append(point)

                # Upload when batch is full
                if len(batch_points) >= batch_size:
                    upload_batch()

            stats["successful_files"] += 1

        except Exception as e:
            error_msg = f"ERROR indexing {file_path.name}: {e}"
            if not quiet:
                print(f"  {error_msg}")
            stats["failed_files"] += 1
            stats["errors"].append(error_msg)
            continue

    # Upload any remaining points
    upload_batch()

    # Calculate final stats
    end_time = time.time()
    duration = end_time - start_time
    stats["duration"] = duration
    stats["rate"] = stats["successful_files"] / duration if duration > 0 else 0

    return stats


def print_summary(stats: Dict[str, int], quiet: bool = False):
    """Print indexing summary"""
    if quiet:
        return

    print("\n" + "=" * 60)
    print("Indexing Complete!")
    print("=" * 60)
    print(f"Files processed: {stats['successful_files']}/{stats['total_files']}")
    print(f"Chunks indexed: {stats['successful_chunks']}")
    print(f"Failed files: {stats['failed_files']}")
    if stats['failed_chunks'] > 0:
        print(f"Failed chunks: {stats['failed_chunks']}")
    print(f"Duration: {stats['duration']:.2f}s")
    if stats['duration'] > 0:
        rate = stats['successful_files'] / stats['duration']
        print(f"Rate: {rate:.2f} files/s")

    if stats['errors']:
        print("\nErrors encountered:")
        for error in stats['errors'][:5]:  # Show first 5 errors
            print(f"  - {error}")
        if len(stats['errors']) > 5:
            print(f"  ... and {len(stats['errors']) - 5} more errors")

    print("\nYou can now use the chatbot to query the textbook content.")


def main():
    """Parse arguments and run indexing"""
    parser = argparse.ArgumentParser(
        description="Index textbook content into Qdrant Cloud",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python backend/scripts/index_content.py --source book-source/docs
    python backend/scripts/index_content.py --source /path/to/docs --batch-size 64
    python backend/scripts/index_content.py --source docs --chunk-size 1500 --chunk-overlap 300
        """
    )

    parser.add_argument(
        '--source',
        type=Path,
        required=True,
        help='Path to documentation source directory'
    )
    parser.add_argument(
        '--batch-size',
        type=int,
        default=32,
        help='Number of embeddings to upload in a batch (default: 32)'
    )
    parser.add_argument(
        '--chunk-size',
        type=int,
        default=1000,
        help='Maximum characters per chunk (default: 1000)'
    )
    parser.add_argument(
        '--chunk-overlap',
        type=int,
        default=200,
        help='Overlap between chunks in characters (default: 200)'
    )
    parser.add_argument(
        '--quiet',
        action='store_true',
        help='Suppress progress output'
    )

    args = parser.parse_args()

    # Validate chunk parameters
    if args.chunk_size < 100:
        print("ERROR: --chunk-size must be at least 100")
        sys.exit(1)
    if args.chunk_overlap >= args.chunk_size:
        print("ERROR: --chunk-overlap must be less than --chunk-size")
        sys.exit(1)
    if args.batch_size < 1:
        print("ERROR: --batch-size must be at least 1")
        sys.exit(1)

    try:
        stats = index_documents(
            source_path=args.source,
            batch_size=args.batch_size,
            chunk_size=args.chunk_size,
            chunk_overlap=args.chunk_overlap,
            quiet=args.quiet
        )
        print_summary(stats, quiet=args.quiet)

        # Exit with error if any files failed
        if stats['failed_files'] > 0:
            sys.exit(1)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(130)
    except Exception as e:
        print(f"\nERROR: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
