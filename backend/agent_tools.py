import os
from typing_extensions import TypedDict
from agents import function_tool
from qdrant_client import QdrantClient
from google import genai
from dotenv import load_dotenv

load_dotenv()

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
    
    # Generate embedding for query
    try:
        response = client.models.embed_content(
            model="text-embedding-004",
            contents=query_text
        )
        query_vector = response.embeddings[0].values
    except Exception as e:
        return "Error generating query embedding."

    # Search Qdrant
    try:
        # Using query_points compatible with newer Qdrant clients
        results_obj = qdrant.query_points(
            collection_name=COLLECTION_NAME,
            query=query_vector,
            limit=3
        )
        hits = results_obj.points
    except Exception as e:
        return "Error searching knowledge base."

    results = []
    for hit in hits:
        results.append(f"Source: {hit.payload['title']}\nContent: {hit.payload['content']}")

    

    if not results:
        return "No relevant information found in the knowledge base."

    return "\n\n".join(results)