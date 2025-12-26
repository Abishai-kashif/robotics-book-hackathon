"""
Text splitting utilities for content chunking
"""
import re
from typing import List, Tuple
from dataclasses import dataclass

@dataclass
class TextChunk:
    text: str
    start_index: int
    end_index: int
    metadata: dict

class TextSplitter:
    """
    Utility class for splitting text into chunks of specified size
    """

    def __init__(self, chunk_size: int = 1000, chunk_overlap: int = 200, separators: List[str] = None):
        """
        Initialize the text splitter

        Args:
            chunk_size: Maximum size of each chunk
            chunk_overlap: Overlap between chunks to maintain context
            separators: List of separators to use for splitting (in order of preference)
        """
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap

        if separators is None:
            # Default separators in order of preference
            self.separators = ["\n\n", "\n", " ", ""]
        else:
            self.separators = separators

    def split_text(self, text: str) -> List[TextChunk]:
        """
        Split text into chunks based on the specified parameters
        """
        return self._split_text_recursive(text, 0)

    def _split_text_recursive(self, text: str, start_index: int) -> List[TextChunk]:
        """
        Recursively split text using the separators in order of preference
        """
        if len(text) <= self.chunk_size:
            return [TextChunk(text=text, start_index=start_index, end_index=start_index + len(text), metadata={})]

        # Try each separator in order
        for separator in self.separators:
            chunks = self._split_by_separator(text, separator)
            if chunks:
                # Process each chunk
                result = []
                for i, chunk in enumerate(chunks):
                    # Calculate the actual position in the original text
                    chunk_start = start_index + text.find(chunk)
                    if i > 0:
                        # Account for previous chunks to get correct position
                        prev_chunks = ''.join(chunks[:i])
                        chunk_start = start_index + len(prev_chunks)
                        # Adjust for the separator we split on
                        if separator:
                            chunk_start += len(separator)

                    if len(chunk) <= self.chunk_size:
                        # Chunk is small enough, add it
                        result.append(
                            TextChunk(
                                text=chunk,
                                start_index=chunk_start,
                                end_index=chunk_start + len(chunk),
                                metadata={}
                            )
                        )
                    else:
                        # Chunk is still too large, split recursively
                        result.extend(
                            self._split_text_recursive(chunk, chunk_start)
                        )
                return result

        # If no separator worked, split by character count
        return self._split_by_size(text, start_index)

    def _split_by_separator(self, text: str, separator: str) -> List[str]:
        """
        Split text by a specific separator
        """
        if separator == "":
            # Split into individual characters
            return [text[i:i+1] for i in range(len(text))]

        # Split by the separator
        chunks = text.split(separator)

        # If separator is not a space, we need to add it back to all but the last chunk
        if separator != " " and separator != "\n" and separator != "\n\n":
            # Add separator back to all but the last chunk
            for i in range(len(chunks) - 1):
                chunks[i] += separator
        elif separator == " " or separator == "\n":
            # For space or newline separators, add them back appropriately
            for i in range(len(chunks) - 1):
                chunks[i] += separator

        # Filter out empty chunks
        return [chunk for chunk in chunks if chunk.strip()]

    def _split_by_size(self, text: str, start_index: int) -> List[TextChunk]:
        """
        Split text by character count as a fallback
        """
        chunks = []
        start = 0

        while start < len(text):
            end = start + self.chunk_size

            # If this isn't the last chunk, try to find a good break point
            if end < len(text):
                # Look for a space to break on
                for i in range(end, start, -1):
                    if text[i] == ' ':
                        end = i
                        break
                else:
                    # If no space found, just break at the limit
                    pass

            chunk_text = text[start:end]
            chunks.append(
                TextChunk(
                    text=chunk_text,
                    start_index=start_index + start,
                    end_index=start_index + end,
                    metadata={}
                )
            )

            # Move start position, accounting for overlap
            start = end - self.chunk_overlap if self.chunk_overlap < end else end

            # Ensure we make progress to avoid infinite loops
            if start <= end - self.chunk_size:
                start = end

        return chunks

class MarkdownTextSplitter(TextSplitter):
    """
    Specialized text splitter for markdown content
    """

    def __init__(self, chunk_size: int = 1000, chunk_overlap: int = 200):
        # Separators specific to markdown, in order of preference
        separators = [
            # First, try to split by sections
            "\n## ",  # H2 headers
            "\n### ", # H3 headers
            "\n#### ", # H4 headers
            "\n##### ", # H5 headers
            "\n###### ", # H6 headers
            "\n# ",   # H1 headers
            "\n\n",   # Paragraph breaks
            "\n",     # Newlines
            " ",      # Spaces
            ""        # Characters as last resort
        ]
        super().__init__(chunk_size, chunk_overlap, separators)

    def _split_by_separator(self, text: str, separator: str) -> List[str]:
        """
        Special handling for markdown separators
        """
        if separator.startswith("\n") and len(separator) > 1:
            # This is a markdown header separator, preserve the header in the next chunk
            parts = []
            start = 0

            while True:
                pos = text.find(separator, start)
                if pos == -1:
                    parts.append(text[start:])
                    break
                # Include the separator in the next chunk to preserve the header
                parts.append(text[start:pos])
                start = pos
                break_next = False
                # Find the end of this header line
                for i in range(pos, len(text)):
                    if text[i] == '\n':
                        start = i
                        break
                else:
                    break

            return [part for part in parts if part.strip()]
        else:
            return super()._split_by_separator(text, separator)

def split_text_by_size(text: str, chunk_size: int, overlap: int = 0) -> List[str]:
    """
    Simple function to split text by size with optional overlap
    """
    if len(text) <= chunk_size:
        return [text]

    chunks = []
    start = 0

    while start < len(text):
        end = min(start + chunk_size, len(text))
        chunks.append(text[start:end])

        if start + chunk_size >= len(text):
            break

        start = end - overlap if overlap < end else end

    return chunks

def split_text_by_sentences(text: str, max_chunk_size: int) -> List[str]:
    """
    Split text by sentences while respecting max chunk size
    """
    # Split by sentence endings
    sentences = re.split(r'(?<=[.!?])\s+', text)

    chunks = []
    current_chunk = ""

    for sentence in sentences:
        # Check if adding this sentence would exceed the limit
        if len(current_chunk) + len(sentence) <= max_chunk_size:
            current_chunk += sentence + " "
        else:
            # If current chunk is not empty, save it
            if current_chunk.strip():
                chunks.append(current_chunk.strip())
            # If the sentence itself is longer than max_chunk_size, split it
            if len(sentence) > max_chunk_size:
                sub_chunks = split_text_by_size(sentence, max_chunk_size, 0)
                chunks.extend(sub_chunks[:-1])  # Add all but the last sub-chunk
                current_chunk = sub_chunks[-1] + " "  # Start new chunk with the last sub-chunk
            else:
                current_chunk = sentence + " "

    # Add the final chunk if it's not empty
    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    return chunks