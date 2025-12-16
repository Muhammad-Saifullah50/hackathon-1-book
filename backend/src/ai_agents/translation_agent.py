# backend/src/ai_agents/translation_agent.py
"""
Urdu translation agent using OpenAI Agents SDK with Gemini model.

Translates page content from English to Urdu while preserving
code blocks, images, links, and markdown structure.
"""

import re
import time
import logging

from agents import Agent, Runner

# Import the Gemini model from existing setup
from models.gemini import gemini_model, config as gemini_config

logger = logging.getLogger(__name__)


# Agent instructions for Urdu translation
TRANSLATION_INSTRUCTIONS = """You are an expert Urdu translator specializing in technical and educational content.

TRANSLATION RULES:
1. Translate all prose text from English to Urdu
2. PRESERVE EXACTLY (do not translate):
   - Code blocks (```...```)
   - Inline code (`...`)
   - Image references (![...](...))
   - URLs and links
   - Variable names and technical identifiers
3. Technical terms handling:
   - Keep technical terms in English with Urdu transliteration in parentheses where helpful
   - Example: "ROS 2 (آر او ایس 2)" for first occurrence
4. Maintain markdown structure:
   - Headings (#, ##, ###)
   - Lists (-, *, 1.)
   - Bold (**), italic (*)
   - Tables (|...|)
5. Output must be valid markdown

QUALITY STANDARDS:
- Use formal Urdu (فصیح اردو)
- Maintain technical accuracy
- Preserve all formatting
"""


def extract_preservable_blocks(content: str) -> tuple[str, list[tuple[str, str]]]:
    """
    Extract code blocks, images, and links, replacing with placeholders.

    Args:
        content: Original markdown content

    Returns:
        Tuple of (modified_content, list_of_preserved_blocks)
    """
    preserved = []
    modified_content = content

    # Preserve fenced code blocks (```...```)
    code_pattern = r'```[\s\S]*?```'
    code_matches = list(re.finditer(code_pattern, modified_content))

    # Process in reverse order to maintain positions
    for i, match in enumerate(reversed(code_matches)):
        idx = len(code_matches) - 1 - i
        placeholder = f"__CODE_BLOCK_{idx}__"
        preserved.insert(0, (placeholder, match.group()))
        modified_content = modified_content[:match.start()] + placeholder + modified_content[match.end():]

    # Preserve inline code (`...`)
    inline_code_pattern = r'`[^`\n]+`'
    inline_matches = list(re.finditer(inline_code_pattern, modified_content))

    for i, match in enumerate(reversed(inline_matches)):
        idx = len(preserved) + len(inline_matches) - 1 - i
        placeholder = f"__INLINE_CODE_{idx}__"
        preserved.insert(0, (placeholder, match.group()))
        modified_content = modified_content[:match.start()] + placeholder + modified_content[match.end():]

    # Preserve images ![alt](url)
    img_pattern = r'!\[.*?\]\([^)]+\)'
    img_matches = list(re.finditer(img_pattern, modified_content))

    for i, match in enumerate(reversed(img_matches)):
        idx = len(preserved) + len(img_matches) - 1 - i
        placeholder = f"__IMAGE_{idx}__"
        preserved.insert(0, (placeholder, match.group()))
        modified_content = modified_content[:match.start()] + placeholder + modified_content[match.end():]

    # Preserve links [text](url) but NOT images (already handled)
    link_pattern = r'(?<!!)\[([^\]]+)\]\(([^)]+)\)'
    link_matches = list(re.finditer(link_pattern, modified_content))

    for i, match in enumerate(reversed(link_matches)):
        idx = len(preserved) + len(link_matches) - 1 - i
        placeholder = f"__LINK_{idx}__"
        preserved.insert(0, (placeholder, match.group()))
        modified_content = modified_content[:match.start()] + placeholder + modified_content[match.end():]

    return modified_content, preserved


def restore_preserved_blocks(content: str, preserved: list[tuple[str, str]]) -> str:
    """
    Restore preserved blocks from placeholders.

    Args:
        content: Content with placeholders
        preserved: List of (placeholder, original) tuples

    Returns:
        Content with original blocks restored
    """
    result = content
    for placeholder, original in preserved:
        result = result.replace(placeholder, original)
    return result


async def translate_to_urdu(
    content: str,
    timeout_seconds: int = 30
) -> tuple[str, int]:
    """
    Translate content from English to Urdu using the AI agent.

    Args:
        content: Original markdown content in English
        timeout_seconds: Maximum time for translation

    Returns:
        Tuple of (translated_content, processing_time_ms)

    Raises:
        TimeoutError: If translation exceeds timeout
        Exception: If agent fails
    """
    start_time = time.time()

    # Extract preservable blocks
    modified_content, preserved = extract_preservable_blocks(content)

    logger.info(f"Translating content to Urdu, preserved {len(preserved)} blocks")

    # Create the translation agent
    urdu_translation_agent = Agent(
        name="urdu_translation_agent",
        instructions=TRANSLATION_INSTRUCTIONS + "\n\nIMPORTANT: Preserve all placeholders (like __CODE_BLOCK_0__, __IMAGE_1__, etc.) exactly as they appear.",
        model=gemini_model,
    )

    # Run the agent
    try:
        result = await Runner.run(
            urdu_translation_agent,
            f"Translate this content from English to Urdu:\n\n{modified_content}",
            run_config=gemini_config,
        )

        translated = result.final_output

        # Restore preserved blocks
        final_content = restore_preserved_blocks(translated, preserved)

        processing_time_ms = int((time.time() - start_time) * 1000)

        logger.info(f"Translation completed in {processing_time_ms}ms")

        return final_content, processing_time_ms

    except Exception as e:
        processing_time_ms = int((time.time() - start_time) * 1000)
        logger.error(f"Translation failed after {processing_time_ms}ms: {e}")
        raise
