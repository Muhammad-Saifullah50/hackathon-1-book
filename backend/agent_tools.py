import os
from typing_extensions import TypedDict
from agents import function_tool
from qdrant_client import QdrantClient
from openai import OpenAI

qdrant = QdrantClient(url=os.getenv("QDRANT_URL", "http://localhost:6333"))
openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
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
    
    # Generate embedding for query
    response = openai_client.embeddings.create(
        input=query_text,
        model="text-embedding-3-small"
    )
    query_vector = response.data[0].embedding

    # Search Qdrant
    hits = qdrant.search(
        collection_name=COLLECTION_NAME,
        query_vector=query_vector,
        limit=3
    )

    results = []
    for hit in hits:
        results.append(f"Source: {hit.payload['title']}\nContent: {hit.payload['content']}")

    return "\n\n".join(results)
