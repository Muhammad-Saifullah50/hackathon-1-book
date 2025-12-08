# Data Model: RAG Tutor Agent

## RAG Knowledge Chunk

| Field | Type | Description |
|---|---|---|
| id | UUID | Unique identifier for the chunk |
| content | Text | The actual text content from the book |
| embedding | Vector (List[float]) | Vector representation of the content |
| metadata | JSON | Contextual info (module, chapter, source file) |

## Agent Tools

### `query_knowledge_base`

| Parameter | Type | Description |
|---|---|---|
| query | string | The user's question or search query |
| top_k | int | Number of results to return (default: 3) |

### `simplify_concept`

| Parameter | Type | Description |
|---|---|---|
| concept | string | The text or concept to explain simply |

### `quiz_me`

| Parameter | Type | Description |
|---|---|---|
| topic | string | The topic to generate questions for |

## ChatKit Integration

### Session Response

```json
{
  "client_secret": "sk-..."
}
```

### Contextual Message (Frontend -> Agent)

```json
{
  "role": "user",
  "content": "Explain this: [selected_text]",
  "metadata": {
    "context_type": "selection",
    "source_url": "..."
  }
}
```
