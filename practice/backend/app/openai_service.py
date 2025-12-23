"""
OpenAI service for embeddings and chat completions.
Handles all interactions with OpenAI API.
"""

from openai import OpenAI
from typing import List, Optional
import tiktoken

from .config import get_settings

settings = get_settings()


class OpenAIService:
    """Service for OpenAI API interactions."""

    def __init__(self):
        self.client = OpenAI(api_key=settings.openai_api_key)
        self.embedding_model = settings.openai_embedding_model
        self.chat_model = settings.openai_chat_model
        self.tokenizer = tiktoken.encoding_for_model("gpt-4o-mini")

    def get_embedding(self, text: str) -> List[float]:
        """Get embedding vector for a single text."""
        response = self.client.embeddings.create(
            model=self.embedding_model,
            input=text,
        )
        return response.data[0].embedding

    def get_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """Get embeddings for multiple texts in a single API call."""
        response = self.client.embeddings.create(
            model=self.embedding_model,
            input=texts,
        )
        return [item.embedding for item in response.data]

    def generate_answer(
        self,
        question: str,
        context: str,
        selected_text: Optional[str] = None,
    ) -> str:
        """
        Generate an answer using retrieved context.

        Args:
            question: User's question
            context: Retrieved book content chunks
            selected_text: User-highlighted text (if provided)

        Returns:
            Generated answer string
        """
        # Build the system prompt
        system_prompt = self._build_system_prompt()

        # Build the user message with context
        user_message = self._build_user_message(question, context, selected_text)

        response = self.client.chat.completions.create(
            model=self.chat_model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_message},
            ],
            temperature=0.3,  # Lower temperature for factual responses
            max_tokens=1000,
        )

        return response.choices[0].message.content

    def _build_system_prompt(self) -> str:
        """Build the system prompt for the chatbot."""
        return """You are a helpful assistant for the "Physical AI: A Complete Guide to Embodied Intelligence, Robotics Simulation, and Vision-Language-Action Systems" textbook.

Your role is to answer questions STRICTLY based on the provided book content. Follow these rules:

1. ONLY use information from the provided context to answer questions.
2. If the answer is not in the provided context, respond with: "I cannot find this information in the book."
3. When answering, be concise but thorough.
4. If referencing specific sections, mention the chapter or topic.
5. For code examples, format them properly with markdown code blocks.
6. For technical concepts, provide clear explanations suitable for the book's audience.
7. Never make up information or provide answers from outside the book content.

Remember: You are a book assistant, not a general AI. Stay within the book's scope."""

    def _build_user_message(
        self,
        question: str,
        context: str,
        selected_text: Optional[str] = None,
    ) -> str:
        """Build the user message with context."""
        if selected_text:
            return f"""The user has highlighted the following text from the book:

<selected_text>
{selected_text}
</selected_text>

Based on this selected text, answer the following question:

Question: {question}

Provide a clear, accurate answer using ONLY the selected text above."""
        else:
            return f"""Here is relevant content from the book:

<book_content>
{context}
</book_content>

Based on the book content above, answer the following question:

Question: {question}

Provide a clear, accurate answer using ONLY the book content provided. If the answer is not in the content, say "I cannot find this information in the book." """

    def count_tokens(self, text: str) -> int:
        """Count tokens in text."""
        return len(self.tokenizer.encode(text))

    def truncate_to_token_limit(self, text: str, max_tokens: int) -> str:
        """Truncate text to fit within token limit."""
        tokens = self.tokenizer.encode(text)
        if len(tokens) <= max_tokens:
            return text
        truncated_tokens = tokens[:max_tokens]
        return self.tokenizer.decode(truncated_tokens)


# Singleton instance
_openai_service: Optional[OpenAIService] = None


def get_openai_service() -> OpenAIService:
    """Get or create OpenAI service singleton."""
    global _openai_service
    if _openai_service is None:
        _openai_service = OpenAIService()
    return _openai_service
