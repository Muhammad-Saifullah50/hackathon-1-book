# Research: Urdu Translation Feature

**Feature**: 012-urdu-translation
**Date**: 2025-12-15
**Status**: Complete

## 1. OpenAI Agents SDK for Translation (Gemini Model)

### Decision
Use the OpenAI Agents SDK with a dedicated `urdu_translation_agent` that uses the Gemini model (same as personalization agent) with specialized instructions for Urdu translation with markdown preservation.

### Rationale
- Consistent with existing personalization_agent pattern (uses Gemini)
- Agent instructions can specify markdown preservation rules
- Gemini has strong multilingual support including Urdu
- Cost-effective compared to GPT-4

### Agent Configuration

```python
from agents import Agent

urdu_translation_agent = Agent(
    name="urdu_translation_agent",
    instructions="""You are an expert Urdu translator specializing in technical and educational content.

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
""",
    model="gemini/gemini-2.0-flash",  # Same model as personalization agent
)
```

### Alternatives Considered
1. **GPT-4**: Rejected - personalization already uses Gemini; maintain consistency
2. **Google Translate API**: Rejected - poor handling of markdown, less control over technical terms
3. **DeepL API**: Rejected - limited Urdu support

---

## 2. RTL/LTR Mixed Content Handling

### Decision
Use CSS `direction: rtl` on the translated content container with `direction: ltr` override for code blocks.

### Rationale
- Standard CSS approach for RTL languages
- Code blocks must remain LTR for readability
- Inline code can inherit parent direction safely

### Implementation

```css
/* Translated content container */
.translated-content {
  direction: rtl;
  text-align: right;
  unicode-bidi: embed;
}

/* Code blocks remain LTR */
.translated-content pre,
.translated-content code {
  direction: ltr;
  text-align: left;
  unicode-bidi: isolate;
}

/* Tables remain LTR for code/data */
.translated-content table {
  direction: ltr;
}
```

### Tailwind CSS Classes
```tsx
// Container
className="[direction:rtl] text-right"

// Code blocks override
className="[direction:ltr] text-left"
```

### Alternatives Considered
1. **JavaScript-based direction switching**: Rejected - CSS is simpler and more performant
2. **Font-based RTL**: Rejected - not necessary, CSS direction is sufficient

---

## 3. IP-Based Quota Tracking

### Decision
Extract client IP from `X-Forwarded-For` header (first IP in chain) with fallback to direct connection IP. Hash IP before storage for privacy.

### Rationale
- Platform may be behind reverse proxy/load balancer
- X-Forwarded-For is industry standard for proxy chains
- IP hashing protects user privacy while enabling rate limiting

### Implementation

```python
from fastapi import Request
import hashlib

def get_client_ip(request: Request) -> str:
    """Extract client IP, handling proxy chains."""
    # Check X-Forwarded-For first (common with proxies)
    forwarded = request.headers.get("X-Forwarded-For")
    if forwarded:
        # First IP in chain is the original client
        client_ip = forwarded.split(",")[0].strip()
    else:
        # Direct connection
        client_ip = request.client.host if request.client else "unknown"
    return client_ip

def hash_ip(ip: str) -> str:
    """Hash IP for privacy-preserving storage."""
    return hashlib.sha256(ip.encode()).hexdigest()[:32]
```

### Database Schema Consideration
- Store `ip_hash` (32 chars) instead of raw IP
- Unique constraint on (ip_hash, date) for anonymous quota

### Alternatives Considered
1. **Raw IP storage**: Rejected - privacy concerns
2. **Session-based tracking**: Rejected - easily circumvented by clearing cookies
3. **Browser fingerprinting**: Rejected - overly complex, privacy invasive

---

## 4. 30-Day Cache Expiry with LRU Eviction

### Decision
Store timestamp with each cache entry; check expiry on read; implement LRU eviction when storage limit reached.

### Rationale
- 30-day expiry balances freshness with API cost savings
- LRU eviction prevents localStorage overflow
- Timestamp comparison is lightweight

### Implementation

```typescript
interface TranslatedPageCache {
  content: string;
  originalContentHash: string;
  timestamp: string; // ISO date
  pageUrl: string;
}

const CACHE_PREFIX = 'translated_urdu_';
const CACHE_EXPIRY_DAYS = 30;
const MAX_CACHE_ENTRIES = 50; // Limit to prevent overflow

function isExpired(timestamp: string): boolean {
  const cacheDate = new Date(timestamp);
  const now = new Date();
  const diffDays = (now.getTime() - cacheDate.getTime()) / (1000 * 60 * 60 * 24);
  return diffDays > CACHE_EXPIRY_DAYS;
}

function getCachedTranslation(pageUrl: string): TranslatedPageCache | null {
  const key = `${CACHE_PREFIX}${pageUrl}`;
  const cached = localStorage.getItem(key);
  if (!cached) return null;

  const parsed: TranslatedPageCache = JSON.parse(cached);

  // Check expiry
  if (isExpired(parsed.timestamp)) {
    localStorage.removeItem(key);
    return null;
  }

  return parsed;
}

function evictOldestIfNeeded(): void {
  const entries: { key: string; timestamp: string }[] = [];

  for (let i = 0; i < localStorage.length; i++) {
    const key = localStorage.key(i);
    if (key?.startsWith(CACHE_PREFIX)) {
      const data = JSON.parse(localStorage.getItem(key) || '{}');
      entries.push({ key, timestamp: data.timestamp || '1970-01-01' });
    }
  }

  if (entries.length >= MAX_CACHE_ENTRIES) {
    // Sort by timestamp, remove oldest
    entries.sort((a, b) => new Date(a.timestamp).getTime() - new Date(b.timestamp).getTime());
    localStorage.removeItem(entries[0].key);
  }
}
```

### Alternatives Considered
1. **No expiry**: Rejected - translations may become stale; storage bloat
2. **7-day expiry**: Rejected - too aggressive; frequent re-translations
3. **IndexedDB**: Rejected - overkill for this use case; localStorage sufficient

---

## 5. Content Extraction Pattern

### Decision
Reuse the same DOM extraction pattern from PersonalizationBar for consistency.

### Rationale
- Proven pattern already tested in personalization feature
- Extracts markdown content while excluding UI components
- Handles h1, personalization bar exclusion properly

### Implementation Reference
```typescript
// Same extractPageContent pattern from PersonalizationBar
const extractPageContent = (): string | null => {
  const article = document.querySelector('article');
  if (!article) return null;

  const markdownContent = article.querySelector('.markdown') ||
                         article.querySelector('.theme-doc-markdown');
  if (!markdownContent) return null;

  const clone = markdownContent.cloneNode(true) as HTMLElement;

  // Remove UI elements
  clone.querySelector('h1')?.remove();
  clone.querySelector('.personalization-bar')?.remove();
  clone.querySelector('.translation-bar')?.remove();

  return clone.innerHTML;
};
```

---

## Summary of Decisions

| Topic | Decision | Key Rationale |
|-------|----------|---------------|
| Translation Agent | OpenAI Agents SDK with Gemini model | Consistent with personalization; cost-effective |
| RTL Support | CSS direction property | Standard, performant, code blocks stay LTR |
| IP Tracking | X-Forwarded-For with IP hashing | Privacy-preserving, proxy-aware |
| Cache Expiry | 30-day with LRU eviction | Balance freshness with cost savings |
| Content Extraction | Reuse PersonalizationBar pattern | Proven, consistent |

All research topics resolved. Ready for Phase 1 design.
