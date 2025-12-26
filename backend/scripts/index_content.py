"""
Script to index textbook content into the vector database.
Run this script to populate Qdrant with embeddings from the docs folder.
"""
import os
import sys
import re
import hashlib
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from sentence_transformers import SentenceTransformer
from src.services.qdrant_service import QdrantService

# Configuration
DOCS_PATH = Path(__file__).parent.parent.parent / "book-source" / "docs"
EMBEDDING_MODEL = "all-MiniLM-L6-v2"
CHUNK_SIZE = 1000  # Characters per chunk
CHUNK_OVERLAP = 200  # Overlap between chunks


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


def chunk_text(text: str, chunk_size: int = CHUNK_SIZE, overlap: int = CHUNK_OVERLAP) -> list:
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


def index_documents():
    """Main function to index all documents."""
    print("=" * 60)
    print("Textbook Content Indexer")
    print("=" * 60)

    # Initialize services
    print("\nInitializing embedding model...")
    encoder = SentenceTransformer(EMBEDDING_MODEL)
    print(f"Loaded model: {EMBEDDING_MODEL}")

    print("\nInitializing Qdrant service...")
    qdrant_service = QdrantService()
    # Force client initialization
    _ = qdrant_service.client
    print("Qdrant service ready")

    # Find all markdown files
    if not DOCS_PATH.exists():
        print(f"ERROR: Docs path not found: {DOCS_PATH}")
        return

    md_files = list(DOCS_PATH.glob("**/*.md"))
    print(f"\nFound {len(md_files)} markdown files to index")

    # Skip template files
    md_files = [f for f in md_files if 'template' not in str(f).lower()]
    print(f"After filtering templates: {len(md_files)} files")

    total_chunks = 0
    indexed_files = 0

    for file_path in md_files:
        try:
            # Read file content
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            if len(content.strip()) < 100:
                print(f"  Skipping (too short): {file_path.name}")
                continue

            # Extract metadata
            title = extract_title_from_markdown(content, file_path.stem)
            metadata = extract_metadata_from_frontmatter(content)

            # Clean and chunk the content
            cleaned_content = clean_markdown(content)
            chunks = chunk_text(cleaned_content)

            if not chunks:
                print(f"  Skipping (no content after cleaning): {file_path.name}")
                continue

            # Calculate relative path for source_path
            relative_path = file_path.relative_to(DOCS_PATH)
            source_path = f"/docs/{str(relative_path).replace(os.sep, '/')}"

            print(f"\nIndexing: {title}")
            print(f"  Source: {source_path}")
            print(f"  Chunks: {len(chunks)}")

            # Index each chunk
            for i, chunk in enumerate(chunks):
                content_id = generate_content_id(source_path, i)

                # Generate embedding
                embedding = encoder.encode(chunk).tolist()

                # Prepare payload
                payload = {
                    "title": title,
                    "body": chunk,
                    "source_path": source_path,
                    "metadata": {
                        **metadata,
                        "chunk_index": i,
                        "total_chunks": len(chunks),
                        "filename": file_path.name
                    }
                }

                # Store in Qdrant
                qdrant_service.store_embeddings(
                    content_id=content_id,
                    embedding=embedding,
                    payload=payload
                )

                total_chunks += 1

            indexed_files += 1

        except Exception as e:
            print(f"  ERROR indexing {file_path.name}: {e}")
            continue

    print("\n" + "=" * 60)
    print("Indexing Complete!")
    print("=" * 60)
    print(f"Files indexed: {indexed_files}")
    print(f"Total chunks: {total_chunks}")
    print("\nYou can now use the chatbot to query the textbook content.")


if __name__ == "__main__":
    index_documents()
