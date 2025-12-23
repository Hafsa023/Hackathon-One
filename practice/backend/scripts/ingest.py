"""
Ingestion script for processing book markdown files and storing in Qdrant.

This script:
1. Reads all markdown files from the docs directory
2. Parses and cleans the content
3. Chunks text into 300-500 word segments with overlap
4. Uploads to Qdrant (embeddings computed server-side via FastEmbed)

Usage:
    python -m scripts.ingest [--force] [--docs-path PATH]
"""

import os
import re
import sys
import argparse
from pathlib import Path
from typing import List, Dict, Optional
import yaml

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.config import get_settings
from app.qdrant_client import get_qdrant_service

settings = get_settings()


class MarkdownProcessor:
    """Processes markdown files into clean text chunks."""

    def __init__(self, chunk_size: int = 400, chunk_overlap: int = 50):
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap

    def extract_frontmatter(self, content: str) -> tuple[Dict, str]:
        """Extract YAML frontmatter from markdown."""
        frontmatter = {}
        body = content

        if content.startswith("---"):
            parts = content.split("---", 2)
            if len(parts) >= 3:
                try:
                    frontmatter = yaml.safe_load(parts[1]) or {}
                except yaml.YAMLError:
                    pass
                body = parts[2]

        return frontmatter, body

    def clean_markdown(self, text: str) -> str:
        """Remove markdown syntax while preserving content meaning."""
        # Remove code blocks but keep a marker
        text = re.sub(r'```[\w]*\n(.*?)\n```', r'[Code Example: \1]', text, flags=re.DOTALL)

        # Remove inline code backticks
        text = re.sub(r'`([^`]+)`', r'\1', text)

        # Convert headers to plain text with markers
        text = re.sub(r'^#{1,6}\s+(.+)$', r'\n\1\n', text, flags=re.MULTILINE)

        # Remove image references but keep alt text
        text = re.sub(r'!\[([^\]]*)\]\([^)]+\)', r'[Image: \1]', text)

        # Remove links but keep text
        text = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', text)

        # Remove HTML tags
        text = re.sub(r'<[^>]+>', '', text)

        # Remove admonition syntax (:::note, :::tip, etc.)
        text = re.sub(r':::\w+', '', text)
        text = re.sub(r':::', '', text)

        # Remove bold/italic markers
        text = re.sub(r'\*\*([^*]+)\*\*', r'\1', text)
        text = re.sub(r'\*([^*]+)\*', r'\1', text)
        text = re.sub(r'__([^_]+)__', r'\1', text)
        text = re.sub(r'_([^_]+)_', r'\1', text)

        # Remove horizontal rules
        text = re.sub(r'^[-*_]{3,}$', '', text, flags=re.MULTILINE)

        # Clean up extra whitespace
        text = re.sub(r'\n{3,}', '\n\n', text)
        text = re.sub(r' {2,}', ' ', text)

        return text.strip()

    def extract_sections(self, content: str) -> List[Dict]:
        """Extract sections based on headers."""
        sections = []
        current_section = {"title": "Introduction", "content": []}

        lines = content.split('\n')

        for line in lines:
            header_match = re.match(r'^(#{1,3})\s+(.+)$', line)
            if header_match:
                if current_section["content"]:
                    sections.append({
                        "title": current_section["title"],
                        "content": '\n'.join(current_section["content"]).strip()
                    })
                current_section = {"title": header_match.group(2), "content": []}
            else:
                current_section["content"].append(line)

        if current_section["content"]:
            sections.append({
                "title": current_section["title"],
                "content": '\n'.join(current_section["content"]).strip()
            })

        return sections

    def chunk_text(self, text: str, section: str = "") -> List[Dict]:
        """Split text into chunks of approximately chunk_size words."""
        words = text.split()

        if len(words) <= self.chunk_size:
            return [{"text": text, "section": section}] if text.strip() else []

        chunks = []
        start = 0

        while start < len(words):
            end = min(start + self.chunk_size, len(words))

            if end < len(words):
                chunk_words = words[start:end]
                chunk_text = ' '.join(chunk_words)

                for ending in ['. ', '! ', '? ', '.\n', '!\n', '?\n']:
                    last_boundary = chunk_text.rfind(ending)
                    if last_boundary > len(chunk_text) * 0.7:
                        chunk_text = chunk_text[:last_boundary + 1]
                        end = start + len(chunk_text.split())
                        break

            chunk_text = ' '.join(words[start:end])

            if chunk_text.strip():
                chunks.append({
                    "text": chunk_text.strip(),
                    "section": section,
                })

            start = end - self.chunk_overlap if end < len(words) else end

        return chunks

    def process_file(self, file_path: Path) -> List[Dict]:
        """Process a single markdown file into chunks."""
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        frontmatter, body = self.extract_frontmatter(content)
        chapter_title = frontmatter.get('title', file_path.stem.replace('-', ' ').title())
        sections = self.extract_sections(body)

        all_chunks = []
        position = 0

        for section in sections:
            cleaned_content = self.clean_markdown(section["content"])

            if not cleaned_content.strip():
                continue

            section_chunks = self.chunk_text(cleaned_content, section["title"])

            for chunk in section_chunks:
                all_chunks.append({
                    "text": chunk["text"],
                    "chapter": chapter_title,
                    "section": chunk["section"],
                    "position": position,
                    "source_file": str(file_path.name),
                })
                position += 1

        return all_chunks


class BookIngester:
    """Main class for ingesting book content into Qdrant."""

    def __init__(self, docs_path: str):
        self.docs_path = Path(docs_path)
        self.processor = MarkdownProcessor(
            chunk_size=settings.chunk_size,
            chunk_overlap=settings.chunk_overlap,
        )
        self.qdrant = get_qdrant_service()

    def find_markdown_files(self) -> List[Path]:
        """Find all markdown files in the docs directory."""
        md_files = []

        for pattern in ['**/*.md', '**/*.mdx']:
            md_files.extend(self.docs_path.glob(pattern))

        md_files = [f for f in md_files if not f.name.startswith('_')]

        return sorted(md_files)

    def ingest(self, force: bool = False):
        """Run the full ingestion pipeline."""
        print(f"Starting ingestion from: {self.docs_path}")

        if force:
            print("Force flag set - recreating collection...")
            try:
                self.qdrant.delete_collection()
            except Exception:
                pass

        self.qdrant.ensure_collection_exists()

        md_files = self.find_markdown_files()
        print(f"Found {len(md_files)} markdown files")

        all_chunks = []
        for file_path in md_files:
            print(f"Processing: {file_path.name}")
            chunks = self.processor.process_file(file_path)
            all_chunks.extend(chunks)
            print(f"  - Generated {len(chunks)} chunks")

        print(f"\nTotal chunks: {len(all_chunks)}")

        # Upload to Qdrant using add method (embeddings computed server-side)
        print("\nUploading to Qdrant (with server-side embeddings)...")

        texts = [chunk["text"] for chunk in all_chunks]
        metadata = [
            {
                "chapter": chunk["chapter"],
                "section": chunk["section"],
                "position": chunk["position"],
                "word_count": len(chunk["text"].split()),
            }
            for chunk in all_chunks
        ]

        # Upload in batches
        batch_size = 50
        uploaded = 0
        for i in range(0, len(texts), batch_size):
            batch_texts = texts[i:i + batch_size]
            batch_meta = metadata[i:i + batch_size]

            print(f"  Uploading batch {i // batch_size + 1}/{(len(texts) + batch_size - 1) // batch_size}")

            self.qdrant.add_texts(batch_texts, batch_meta)
            uploaded += len(batch_texts)

        print(f"\nUploaded {uploaded} chunks to Qdrant")

        print("\n" + "=" * 50)
        print("INGESTION COMPLETE")
        print("=" * 50)
        print(f"Files processed: {len(md_files)}")
        print(f"Total chunks: {len(all_chunks)}")
        print(f"Chunks uploaded: {uploaded}")

        info = self.qdrant.get_collection_info()
        print(f"\nCollection info:")
        for key, value in info.items():
            print(f"  {key}: {value}")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Ingest book content into Qdrant for RAG chatbot"
    )
    parser.add_argument(
        "--docs-path",
        default="../docs",
        help="Path to the docs directory (default: ../docs)"
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Force re-indexing by deleting existing collection"
    )

    args = parser.parse_args()

    script_dir = Path(__file__).parent
    docs_path = (script_dir / args.docs_path).resolve()

    if not docs_path.exists():
        print(f"Error: Docs path does not exist: {docs_path}")
        sys.exit(1)

    ingester = BookIngester(str(docs_path))
    ingester.ingest(force=args.force)


if __name__ == "__main__":
    main()
