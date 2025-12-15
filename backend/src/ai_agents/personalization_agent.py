# backend/src/ai_agents/personalization_agent.py
"""
Personalization agent using OpenAI Agents SDK with Gemini model.

Personalizes page content based on user learning profile using
dynamic instructions that adapt to each user's tech_background,
learning_mode, learning_speed, and preferred_language.
"""

import re
import time
import logging
from dataclasses import dataclass
from typing import Optional

from agents import Agent, Runner

# Import the Gemini model from existing setup
from models.gemini import gemini_model, config as gemini_config

logger = logging.getLogger(__name__)


@dataclass
class PersonalizationContext:
    """Context for personalization agent with user profile attributes."""
    user_id: str
    tech_background: Optional[str] = None  # none, beginner, intermediate, advanced
    learning_mode: Optional[str] = None    # visual, reading, hands_on, mixed
    learning_speed: Optional[str] = None   # thorough, balanced, accelerated
    preferred_language: str = "en"         # en, es, zh, ar
    education_level: Optional[str] = None  # high_school, bachelors, masters, phd, self_taught
    primary_goal: Optional[str] = None     # career, research, hobby, education
    focus_area: Optional[str] = None       # hardware, software


def extract_preservable_blocks(content: str) -> tuple[str, list[tuple[str, str]]]:
    """
    Extract code blocks, images, and tables, replacing with placeholders.

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


def build_instructions_for_profile(profile: PersonalizationContext) -> str:
    """
    Build instructions string based on user profile.

    Args:
        profile: User's personalization context

    Returns:
        Instruction string tailored to user profile
    """
    instructions = [
        "You are a content personalizer for a Physical AI and Humanoid Robotics education platform.",
        "Your task is to rewrite the provided markdown content to match the learner's profile.",
        "IMPORTANT: Preserve all placeholders (like __CODE_BLOCK_0__, __IMAGE_1__, etc.) exactly as they appear.",
        "",
        "## Learner Profile:",
    ]

    # Technical background adaptation
    tech_bg = profile.tech_background or "beginner"
    if tech_bg == "none" or tech_bg == "beginner":
        instructions.append("- Technical Level: BEGINNER - Use simple analogies, avoid jargon, explain all acronyms, relate concepts to everyday experiences")
    elif tech_bg == "intermediate":
        instructions.append("- Technical Level: INTERMEDIATE - Balance technical depth with accessibility, assume basic programming knowledge")
    else:  # advanced
        instructions.append("- Technical Level: ADVANCED - Include mathematical formulations, reference research papers, use technical terminology freely")

    # Learning mode adaptation
    learning_mode = profile.learning_mode or "mixed"
    if learning_mode == "visual":
        instructions.append("- Learning Style: VISUAL - Suggest diagrams, use bullet points, describe spatial relationships, reference visual elements")
    elif learning_mode == "hands_on":
        instructions.append("- Learning Style: HANDS-ON - Add practical exercises, suggest experiments, include step-by-step tutorials")
    elif learning_mode == "reading":
        instructions.append("- Learning Style: READING - Provide detailed prose explanations, include references to further reading")
    else:  # mixed
        instructions.append("- Learning Style: MIXED - Balance explanations with practical examples and visual descriptions")

    # Learning speed
    learning_speed = profile.learning_speed or "balanced"
    if learning_speed == "thorough":
        instructions.append("- Pace: THOROUGH - Include detailed explanations, multiple examples, additional context, elaborate on key concepts")
    elif learning_speed == "accelerated":
        instructions.append("- Pace: ACCELERATED - Be concise, focus on key points, skip obvious details, get to the core concepts quickly")
    else:  # balanced
        instructions.append("- Pace: BALANCED - Provide clear explanations without being verbose")

    # Education level context
    education = profile.education_level
    if education == "high_school":
        instructions.append("- Education: HIGH SCHOOL - Avoid assuming advanced math or physics knowledge")
    elif education == "phd":
        instructions.append("- Education: DOCTORAL - Can reference advanced theoretical concepts")

    # Primary goal
    goal = profile.primary_goal
    if goal == "career":
        instructions.append("- Goal: CAREER - Emphasize industry applications and job-relevant skills")
    elif goal == "research":
        instructions.append("- Goal: RESEARCH - Highlight open problems and research directions")
    elif goal == "hobby":
        instructions.append("- Goal: HOBBY - Focus on fun, interesting aspects and DIY possibilities")

    # Focus area
    focus = profile.focus_area
    if focus == "hardware":
        instructions.append("- Focus: HARDWARE - Emphasize physical components, sensors, actuators")
    elif focus == "software":
        instructions.append("- Focus: SOFTWARE - Emphasize algorithms, code, and software architecture")

    # Language
    lang = profile.preferred_language
    if lang != "en":
        lang_names = {"es": "Spanish", "zh": "Chinese", "ar": "Arabic"}
        lang_name = lang_names.get(lang, lang)
        instructions.append(f"- Language: Respond ENTIRELY in {lang_name}")

    # Output rules
    instructions.extend([
        "",
        "## OUTPUT RULES:",
        "1. PRESERVE all placeholders exactly as they appear (e.g., __CODE_BLOCK_0__, __IMAGE_1__)",
        "2. Maintain the same markdown structure and heading hierarchy",
        "3. Only modify explanatory prose paragraphs",
        "4. Output valid markdown only",
        "5. Do NOT add any preamble or explanation - just output the personalized content",
    ])

    return "\n".join(instructions)


async def personalize_content(
    content: str,
    profile: PersonalizationContext,
    timeout_seconds: int = 15
) -> tuple[str, int]:
    """
    Personalize content using the AI agent.

    Args:
        content: Original markdown content
        profile: User's personalization context
        timeout_seconds: Maximum time for personalization

    Returns:
        Tuple of (personalized_content, processing_time_ms)

    Raises:
        TimeoutError: If personalization exceeds timeout
        Exception: If agent fails
    """
    start_time = time.time()

    # Extract preservable blocks
    modified_content, preserved = extract_preservable_blocks(content)

    logger.info(f"Personalizing content for user {profile.user_id}, preserved {len(preserved)} blocks")

    # Build dynamic instructions for this profile
    instructions = build_instructions_for_profile(profile)

    # Create the agent with profile-specific instructions
    personalization_agent = Agent(
        name="Content Personalizer",
        instructions=instructions,
        model=gemini_model,
    )

    # Run the agent
    try:
        result = await Runner.run(
            personalization_agent,
            f"Personalize this content:\n\n{modified_content}",
            run_config=gemini_config,
        )

        personalized = result.final_output

        # Restore preserved blocks
        final_content = restore_preserved_blocks(personalized, preserved)

        processing_time_ms = int((time.time() - start_time) * 1000)

        logger.info(f"Personalization completed in {processing_time_ms}ms")

        return final_content, processing_time_ms

    except Exception as e:
        processing_time_ms = int((time.time() - start_time) * 1000)
        logger.error(f"Personalization failed after {processing_time_ms}ms: {e}")
        raise
