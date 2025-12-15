# Research: Page Content Personalization

**Feature**: 011-personalize-page
**Date**: 2025-12-14
**Status**: Complete

## Research Topics

1. OpenAI Agents SDK - Dynamic Instructions Pattern
2. Markdown Content Preservation Strategy
3. Profile Hashing for Change Detection
4. localStorage Management and LRU Eviction

---

## 1. OpenAI Agents SDK - Dynamic Instructions

### Decision
Use the OpenAI Agents SDK with **dynamic instructions** pattern. The instructions are provided as a function that receives `RunContextWrapper` with user profile data, allowing runtime customization of the agent's behavior.

### Rationale
- Dynamic instructions enable profile-specific personalization without creating multiple agent instances
- The `RunContextWrapper` pattern provides type-safe access to user context
- `Runner.run()` or `Runner.run_sync()` provides straightforward execution with result access

### Implementation Pattern

```python
from dataclasses import dataclass
from agents import Agent, Runner, RunContextWrapper

@dataclass
class PersonalizationContext:
    user_id: str
    tech_background: str  # beginner, intermediate, advanced
    learning_mode: str    # visual, reading, hands_on, mixed
    learning_speed: str   # thorough, balanced, accelerated
    preferred_language: str  # en, es, zh, ar
    education_level: str
    primary_goal: str
    focus_area: str

def build_dynamic_instructions(
    context: RunContextWrapper[PersonalizationContext],
    agent: Agent[PersonalizationContext]
) -> str:
    profile = context.context
    instructions = [
        "You are a content personalizer for a robotics education platform.",
        "Your task is to rewrite the provided markdown content to match the learner's profile.",
        "",
        "## Learner Profile:",
    ]

    # Technical background adaptation
    if profile.tech_background == "beginner":
        instructions.append("- Technical Level: BEGINNER - Use simple analogies, avoid jargon, explain all acronyms")
    elif profile.tech_background == "intermediate":
        instructions.append("- Technical Level: INTERMEDIATE - Balance technical depth with accessibility")
    else:
        instructions.append("- Technical Level: ADVANCED - Include mathematical formulations, cite relevant papers")

    # Learning mode adaptation
    if profile.learning_mode == "visual":
        instructions.append("- Learning Style: VISUAL - Suggest diagrams, use bullet points, add visual descriptions")
    elif profile.learning_mode == "hands_on":
        instructions.append("- Learning Style: HANDS-ON - Add practical exercises, code challenges, real-world examples")
    elif profile.learning_mode == "reading":
        instructions.append("- Learning Style: READING - Provide detailed prose explanations, reference materials")

    # Learning speed
    if profile.learning_speed == "thorough":
        instructions.append("- Pace: THOROUGH - Include detailed explanations, additional context, multiple examples")
    elif profile.learning_speed == "accelerated":
        instructions.append("- Pace: ACCELERATED - Be concise, focus on key points, skip obvious details")

    # Language
    if profile.preferred_language != "en":
        instructions.append(f"- Language: Respond in {profile.preferred_language}")

    # Preservation rules
    instructions.extend([
        "",
        "## CRITICAL RULES:",
        "1. PRESERVE all code blocks exactly as-is (```code``` sections)",
        "2. PRESERVE all images and their markdown syntax (![alt](url))",
        "3. PRESERVE all tables exactly as-is",
        "4. PRESERVE all links exactly as-is",
        "5. Only modify explanatory prose paragraphs",
        "6. Maintain the same markdown structure and heading hierarchy",
        "7. Output valid markdown only",
    ])

    return "\n".join(instructions)

# Create the agent with dynamic instructions
personalization_agent = Agent[PersonalizationContext](
    name="Content Personalizer",
    instructions=build_dynamic_instructions,
)

# Execute personalization
async def personalize_content(content: str, profile: PersonalizationContext) -> str:
    result = await Runner.run(
        personalization_agent,
        f"Personalize this content:\n\n{content}",
        context=profile,
    )
    return result.final_output
```

### Alternatives Considered

| Alternative | Why Rejected |
|------------|--------------|
| Static instructions with profile in prompt | Less clean, harder to maintain, no type safety |
| Multiple agent instances per profile type | Doesn't scale with profile combinations |
| LangChain | Heavier dependency, OpenAI Agents SDK is simpler for this use case |
| Direct OpenAI API | Loses agent abstraction, no built-in context management |

---

## 2. Markdown Content Preservation Strategy

### Decision
Use a **selective personalization** approach that parses markdown into blocks and only sends prose paragraphs to the AI, preserving code blocks, images, and tables unchanged.

### Rationale
- Prevents AI from accidentally modifying or breaking code examples
- Reduces token costs by not processing non-prose content
- Ensures 100% preservation of technical content (SC-003 requirement)

### Implementation Pattern

```python
import re
from typing import List, Tuple

def extract_preservable_blocks(content: str) -> Tuple[str, List[Tuple[str, str]]]:
    """
    Extract code blocks, images, and tables, replacing with placeholders.
    Returns (modified_content, list_of_preserved_blocks)
    """
    preserved = []

    # Preserve fenced code blocks (```...```)
    code_pattern = r'```[\s\S]*?```'
    for i, match in enumerate(re.finditer(code_pattern, content)):
        placeholder = f"__CODE_BLOCK_{i}__"
        preserved.append((placeholder, match.group()))
    content = re.sub(code_pattern, lambda m: f"__CODE_BLOCK_{len(preserved)-1}__", content, count=1)

    # Repeat pattern for each code block found
    while re.search(code_pattern, content):
        match = re.search(code_pattern, content)
        placeholder = f"__CODE_BLOCK_{len(preserved)}__"
        preserved.append((placeholder, match.group()))
        content = content.replace(match.group(), placeholder, 1)

    # Preserve images ![alt](url)
    img_pattern = r'!\[.*?\]\(.*?\)'
    for match in re.finditer(img_pattern, content):
        placeholder = f"__IMAGE_{len(preserved)}__"
        preserved.append((placeholder, match.group()))
        content = content.replace(match.group(), placeholder, 1)

    # Preserve tables (lines starting with |)
    lines = content.split('\n')
    table_lines = []
    in_table = False
    table_start = 0

    for i, line in enumerate(lines):
        if line.strip().startswith('|'):
            if not in_table:
                in_table = True
                table_start = i
        else:
            if in_table:
                table_content = '\n'.join(lines[table_start:i])
                placeholder = f"__TABLE_{len(preserved)}__"
                preserved.append((placeholder, table_content))
                for j in range(table_start, i):
                    lines[j] = placeholder if j == table_start else ""
                in_table = False

    content = '\n'.join(lines)
    return content, preserved

def restore_preserved_blocks(content: str, preserved: List[Tuple[str, str]]) -> str:
    """Restore preserved blocks from placeholders."""
    for placeholder, original in preserved:
        content = content.replace(placeholder, original)
    return content
```

### Alternatives Considered

| Alternative | Why Rejected |
|------------|--------------|
| Send full content with instructions to preserve | AI may still modify code; higher token costs |
| Use markdown AST parser (remark/mdast) | More complex, overkill for this use case |
| Regex-only with prompt instructions | Less reliable preservation guarantee |

---

## 3. Profile Hashing for Change Detection

### Decision
Use **SHA-256 hashing** of serialized profile attributes to detect profile changes. Compare stored hash with current hash to determine if re-personalization is needed.

### Rationale
- SHA-256 is cryptographically secure and collision-resistant
- Deterministic output for same input ensures reliable comparison
- Standard library support in both Python (hashlib) and JavaScript (SubtleCrypto)

### Implementation Pattern

**Python (Backend):**
```python
import hashlib
import json
from typing import Dict, Any

def compute_profile_hash(profile: Dict[str, Any]) -> str:
    """
    Compute SHA-256 hash of profile attributes.
    Only includes personalization-relevant fields.
    """
    relevant_fields = {
        'tech_background': profile.get('tech_background'),
        'learning_mode': profile.get('learning_mode'),
        'learning_speed': profile.get('learning_speed'),
        'preferred_language': profile.get('preferred_language'),
        'education_level': profile.get('education_level'),
        'primary_goal': profile.get('primary_goal'),
        'focus_area': profile.get('focus_area'),
    }
    # Sort keys for deterministic serialization
    serialized = json.dumps(relevant_fields, sort_keys=True)
    return hashlib.sha256(serialized.encode()).hexdigest()
```

**TypeScript (Frontend):**
```typescript
async function computeProfileHash(profile: UserProfile): Promise<string> {
  const relevantFields = {
    tech_background: profile.tech_background,
    learning_mode: profile.learning_mode,
    learning_speed: profile.learning_speed,
    preferred_language: profile.preferred_language,
    education_level: profile.education_level,
    primary_goal: profile.primary_goal,
    focus_area: profile.focus_area,
  };

  const serialized = JSON.stringify(relevantFields, Object.keys(relevantFields).sort());
  const encoder = new TextEncoder();
  const data = encoder.encode(serialized);
  const hashBuffer = await crypto.subtle.digest('SHA-256', data);
  const hashArray = Array.from(new Uint8Array(hashBuffer));
  return hashArray.map(b => b.toString(16).padStart(2, '0')).join('');
}
```

### Alternatives Considered

| Alternative | Why Rejected |
|------------|--------------|
| MD5 | Cryptographically broken, not recommended |
| Simple string concatenation | Not collision-resistant |
| Deep equality comparison | More complex, less efficient for storage |

---

## 4. localStorage Management and LRU Eviction

### Decision
Implement **LRU (Least Recently Used) eviction** with a maximum of 20 cached pages per user. Track access timestamps and evict oldest entries when limit is reached.

### Rationale
- Prevents localStorage bloat (5-10MB browser limit)
- LRU ensures frequently accessed pages remain cached
- 20 pages × ~50KB average = ~1MB, well within limits

### Implementation Pattern

```typescript
interface CachedPage {
  content: string;
  profileHash: string;
  originalContentHash: string;
  timestamp: string;
  pageUrl: string;
  lastAccessed: string;  // For LRU tracking
}

const MAX_CACHED_PAGES = 20;
const CACHE_PREFIX = 'personalized_';

function getCacheKey(pageUrl: string, userId: string): string {
  return `${CACHE_PREFIX}${pageUrl}_${userId}`;
}

function getAllCacheKeys(userId: string): string[] {
  const keys: string[] = [];
  for (let i = 0; i < localStorage.length; i++) {
    const key = localStorage.key(i);
    if (key?.startsWith(CACHE_PREFIX) && key.endsWith(`_${userId}`)) {
      keys.push(key);
    }
  }
  return keys;
}

function evictLRU(userId: string): void {
  const keys = getAllCacheKeys(userId);
  if (keys.length < MAX_CACHED_PAGES) return;

  // Sort by lastAccessed (oldest first)
  const entries = keys.map(key => {
    const data = JSON.parse(localStorage.getItem(key) || '{}') as CachedPage;
    return { key, lastAccessed: data.lastAccessed || data.timestamp };
  }).sort((a, b) => new Date(a.lastAccessed).getTime() - new Date(b.lastAccessed).getTime());

  // Remove oldest entries until under limit
  const toRemove = entries.slice(0, entries.length - MAX_CACHED_PAGES + 1);
  toRemove.forEach(entry => localStorage.removeItem(entry.key));
}

function saveToCache(pageUrl: string, userId: string, data: Omit<CachedPage, 'lastAccessed'>): void {
  evictLRU(userId);
  const key = getCacheKey(pageUrl, userId);
  const entry: CachedPage = {
    ...data,
    lastAccessed: new Date().toISOString(),
  };
  localStorage.setItem(key, JSON.stringify(entry));
}

function getFromCache(pageUrl: string, userId: string): CachedPage | null {
  const key = getCacheKey(pageUrl, userId);
  const data = localStorage.getItem(key);
  if (!data) return null;

  // Update lastAccessed on read
  const entry = JSON.parse(data) as CachedPage;
  entry.lastAccessed = new Date().toISOString();
  localStorage.setItem(key, JSON.stringify(entry));

  return entry;
}

function invalidateAllForUser(userId: string): void {
  getAllCacheKeys(userId).forEach(key => localStorage.removeItem(key));
}
```

### Alternatives Considered

| Alternative | Why Rejected |
|------------|--------------|
| No eviction (unlimited cache) | Risk of hitting localStorage limits |
| FIFO (First In First Out) | May evict frequently used pages |
| Size-based eviction | More complex to implement, less predictable |
| IndexedDB | Overkill for this use case; localStorage is simpler |

---

## Summary

| Topic | Decision | Key Benefit |
|-------|----------|-------------|
| Agent Pattern | Dynamic instructions with RunContextWrapper | Profile-specific behavior at runtime |
| Content Preservation | Extract/restore preservable blocks | 100% code block preservation |
| Profile Hashing | SHA-256 of serialized attributes | Reliable change detection |
| Cache Management | LRU eviction with 20-page limit | Prevents localStorage bloat |

All research topics resolved. Ready for Phase 1: Design & Contracts.
