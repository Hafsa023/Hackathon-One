"""
LLM service for Claude via TryBons/Bonsai proxy.
"""

from openai import OpenAI
from typing import Optional, Generator

from .config import get_settings

settings = get_settings()


class LLMService:
    """Service for Claude chat via TryBons/Bonsai."""

    def __init__(self):
        self.client = OpenAI(
            api_key=settings.trybons_api_key,
            base_url=settings.trybons_base_url,
        )
        self.chat_model = settings.trybons_chat_model
        print(f"LLM service initialized (Claude via TryBons: {self.chat_model})")

    def generate_answer(
        self,
        question: str,
        context: str,
        selected_text: Optional[str] = None,
    ) -> str:
        """
        Generate an answer using retrieved context via Claude.

        Args:
            question: User's question
            context: Retrieved book content chunks
            selected_text: User-highlighted text (if provided)

        Returns:
            Generated answer string
        """
        system_prompt = self._build_system_prompt()
        user_message = self._build_user_message(question, context, selected_text)

        response = self.client.chat.completions.create(
            model=self.chat_model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_message},
            ],
            temperature=0.3,
            max_tokens=1000,
        )

        return response.choices[0].message.content

    def generate_answer_stream(
        self,
        question: str,
        context: str,
        selected_text: Optional[str] = None,
    ) -> Generator[str, None, None]:
        """
        Generate an answer using retrieved context via Claude with streaming.

        Args:
            question: User's question
            context: Retrieved book content chunks
            selected_text: User-highlighted text (if provided)

        Yields:
            Chunks of the generated answer
        """
        system_prompt = self._build_system_prompt()
        user_message = self._build_user_message(question, context, selected_text)

        response = self.client.chat.completions.create(
            model=self.chat_model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_message},
            ],
            temperature=0.3,
            max_tokens=1000,
            stream=True,
        )

        for chunk in response:
            if chunk.choices[0].delta.content:
                yield chunk.choices[0].delta.content

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
        """Estimate token count (rough approximation)."""
        return len(text) // 4

    def truncate_to_token_limit(self, text: str, max_tokens: int) -> str:
        """Truncate text to fit within token limit."""
        estimated_tokens = self.count_tokens(text)
        if estimated_tokens <= max_tokens:
            return text
        max_chars = max_tokens * 4
        return text[:max_chars]


# Singleton instance
_llm_service: Optional[LLMService] = None


def get_llm_service() -> LLMService:
    """Get or create LLM service singleton."""
    global _llm_service
    if _llm_service is None:
        _llm_service = LLMService()
    return _llm_service
