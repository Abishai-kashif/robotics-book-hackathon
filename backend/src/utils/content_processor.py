"""
Content processing pipeline for converting textbook content to vector embeddings
"""
import os
import logging
from typing import List, Dict, Any
from pathlib import Path
import markdown
from bs4 import BeautifulSoup
import re

from ..models.embedding import TextbookContent
from ..services.qdrant_service import QdrantService
from ..services.rag_service import RAGService

logger = logging.getLogger(__name__)

class ContentProcessor:
    def __init__(self, qdrant_service: QdrantService):
        self.qdrant_service = qdrant_service
        self.rag_service = RAGService(qdrant_service)

    def process_textbook_content(self, source_path: str, chunk_size: int = 1000) -> List[TextbookContent]:
        """
        Process textbook content from various formats and convert to chunks for embedding
        """
        content_chunks = []

        # Determine file type and process accordingly
        file_path = Path(source_path)

        if file_path.suffix.lower() == '.md':
            content_chunks = self._process_markdown_file(file_path, chunk_size)
        elif file_path.suffix.lower() == '.txt':
            content_chunks = self._process_text_file(file_path, chunk_size)
        elif file_path.suffix.lower() in ['.html', '.htm']:
            content_chunks = self._process_html_file(file_path, chunk_size)
        else:
            # Try to process as text regardless of extension
            content_chunks = self._process_text_file(file_path, chunk_size)

        logger.info(f"Processed {len(content_chunks)} content chunks from {source_path}")
        return content_chunks

    def _process_markdown_file(self, file_path: Path, chunk_size: int) -> List[TextbookContent]:
        """
        Process a markdown file and split into chunks
        """
        with open(file_path, 'r', encoding='utf-8') as file:
            content = file.read()

        # Convert markdown to plain text for processing
        html = markdown.markdown(content)
        text = BeautifulSoup(html, 'html.parser').get_text()

        # Extract title from the first heading
        lines = content.split('\n')
        title = ""
        for line in lines:
            if line.startswith('# '):
                title = line[2:].strip()
                break
        if not title:
            title = file_path.stem

        # Split content into chunks
        chunks = self._split_content(text, chunk_size)

        # Create TextbookContent objects
        textbook_contents = []
        for i, chunk in enumerate(chunks):
            content_obj = TextbookContent(
                title=f"{title} - Part {i+1}",
                body=chunk,
                source_path=str(file_path),
                embedding=[],  # Will be filled in later
                metadata={
                    "file_type": "markdown",
                    "original_file": file_path.name,
                    "chunk_index": i,
                    "total_chunks": len(chunks)
                }
            )
            textbook_contents.append(content_obj)

        return textbook_contents

    def _process_text_file(self, file_path: Path, chunk_size: int) -> List[TextbookContent]:
        """
        Process a text file and split into chunks
        """
        with open(file_path, 'r', encoding='utf-8') as file:
            content = file.read()

        title = file_path.stem

        # Split content into chunks
        chunks = self._split_content(content, chunk_size)

        # Create TextbookContent objects
        textbook_contents = []
        for i, chunk in enumerate(chunks):
            content_obj = TextbookContent(
                title=f"{title} - Part {i+1}",
                body=chunk,
                source_path=str(file_path),
                embedding=[],  # Will be filled in later
                metadata={
                    "file_type": "text",
                    "original_file": file_path.name,
                    "chunk_index": i,
                    "total_chunks": len(chunks)
                }
            )
            textbook_contents.append(content_obj)

        return textbook_contents

    def _process_html_file(self, file_path: Path, chunk_size: int) -> List[TextbookContent]:
        """
        Process an HTML file and split into chunks
        """
        with open(file_path, 'r', encoding='utf-8') as file:
            content = file.read()

        soup = BeautifulSoup(content, 'html.parser')

        # Extract title from <title> tag or <h1> tag
        title_tag = soup.find('title') or soup.find('h1')
        title = title_tag.get_text().strip() if title_tag else file_path.stem

        # Extract main text content, preserving some structure
        text = soup.get_text(separator=' ')

        # Clean up extra whitespace
        text = re.sub(r'\s+', ' ', text).strip()

        # Split content into chunks
        chunks = self._split_content(text, chunk_size)

        # Create TextbookContent objects
        textbook_contents = []
        for i, chunk in enumerate(chunks):
            content_obj = TextbookContent(
                title=f"{title} - Part {i+1}",
                body=chunk,
                source_path=str(file_path),
                embedding=[],  # Will be filled in later
                metadata={
                    "file_type": "html",
                    "original_file": file_path.name,
                    "chunk_index": i,
                    "total_chunks": len(chunks)
                }
            )
            textbook_contents.append(content_obj)

        return textbook_contents

    def _split_content(self, content: str, chunk_size: int) -> List[str]:
        """
        Split content into chunks of approximately chunk_size characters
        """
        if len(content) <= chunk_size:
            return [content]

        chunks = []
        paragraphs = content.split('\n\n')

        current_chunk = ""
        for paragraph in paragraphs:
            if len(paragraph) > chunk_size:
                # If a single paragraph is too long, split it by sentences
                sentences = re.split(r'[.!?]+', paragraph)
                temp_chunk = ""
                for sentence in sentences:
                    sentence = sentence.strip()
                    if not sentence:
                        continue
                    if len(temp_chunk) + len(sentence) < chunk_size:
                        temp_chunk += sentence + ". "
                    else:
                        if temp_chunk:
                            chunks.append(temp_chunk.strip())
                        temp_chunk = sentence + ". "
                if temp_chunk:
                    chunks.append(temp_chunk.strip())
            else:
                if len(current_chunk) + len(paragraph) < chunk_size:
                    current_chunk += paragraph + "\n\n"
                else:
                    if current_chunk:
                        chunks.append(current_chunk.strip())
                    current_chunk = paragraph + "\n\n"

        if current_chunk:
            chunks.append(current_chunk.strip())

        # Ensure no chunk exceeds the size limit
        final_chunks = []
        for chunk in chunks:
            if len(chunk) > chunk_size:
                # Further split if needed
                sub_chunks = self._split_long_chunk(chunk, chunk_size)
                final_chunks.extend(sub_chunks)
            else:
                final_chunks.append(chunk)

        return final_chunks

    def _split_long_chunk(self, chunk: str, chunk_size: int) -> List[str]:
        """
        Further split a chunk that's still too large
        """
        if len(chunk) <= chunk_size:
            return [chunk]

        # Split by sentences
        sentences = re.split(r'[.!?]+', chunk)
        chunks = []
        current_chunk = ""

        for sentence in sentences:
            sentence = sentence.strip() + "."
            if not sentence.strip() or sentence.strip() == ".":
                continue
            if len(current_chunk) + len(sentence) < chunk_size:
                current_chunk += sentence + " "
            else:
                if current_chunk:
                    chunks.append(current_chunk.strip())
                current_chunk = sentence + " "

        if current_chunk:
            chunks.append(current_chunk.strip())

        return chunks

    async def process_and_store_content(self, source_path: str, chunk_size: int = 1000):
        """
        Process content and store embeddings in Qdrant
        """
        logger.info(f"Processing and storing content from: {source_path}")

        content_chunks = self.process_textbook_content(source_path, chunk_size)

        for content_chunk in content_chunks:
            # Generate embeddings and store in Qdrant
            await self.rag_service.store_content_embeddings(content_chunk)

        logger.info(f"Successfully processed and stored {len(content_chunks)} content chunks")