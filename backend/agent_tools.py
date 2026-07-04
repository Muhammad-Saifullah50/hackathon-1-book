import os
import sys
from typing_extensions import TypedDict
from agents import function_tool
from qdrant_client import QdrantClient
from google import genai
from dotenv import load_dotenv

load_dotenv()

RAG_LOG = True  # set False to silence


def rag_log(msg: str):
    """Bypasses uvicorn's logging dictConfig which suppresses custom loggers."""
    if RAG_LOG:
        print(f"[RAG] {msg}", file=sys.stderr, flush=True)


QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", None)
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
client = genai.Client(api_key=GEMINI_API_KEY)

COLLECTION_NAME = "rag_tutor_knowledge_base"


class KnowledgeQuery(TypedDict):
    query: str


@function_tool
def query_knowledge_base(args: KnowledgeQuery) -> str:
    """
    Query the knowledge base for relevant information from the textbook.
    Use this tool to answer questions about robotics, ROS 2, and physical AI.
    """
    query_text = args["query"]
    rag_log(f"QUERY: {query_text!r}")

    # Generate embedding for query
    try:
        response = client.models.embed_content(
            model="gemini-embedding-001", contents=query_text
        )
        query_vector = response.embeddings[0].values
        rag_log(f"EMBED: dim={len(query_vector)} sample={query_vector[:5]}")
    except Exception as e:
        rag_log(f"EMBED_ERROR: {e}")
        return "Error generating query embedding."

    # Search Qdrant
    try:
        results_obj = qdrant.query_points(
            collection_name=COLLECTION_NAME, query=query_vector, limit=3
        )
        hits = results_obj.points
        rag_log(f"QDRANT: {len(hits)} hits for {query_text!r}")
    except Exception as e:
        rag_log(f"QDRANT_ERROR: {e}")
        return "Error searching knowledge base."

    results = []
    for i, hit in enumerate(hits):
        title = hit.payload.get("title", "?")
        score = hit.score if hasattr(hit, "score") else "N/A"
        content_preview = hit.payload.get("content", "")[:120]
        rag_log(f"HIT {i}: score={score} title={title!r} preview={content_preview!r}")
        results.append(f"Source: {title}\nContent: {hit.payload['content']}")

    if not results:
        rag_log(f"NO_RESULTS: {query_text!r}")
        return "No relevant information found in the knowledge base."

    rag_log(f"RETURN: {len(results)} results for {query_text!r}")
    return "\n\n".join(results)
