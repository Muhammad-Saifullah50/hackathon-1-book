import os
import glob
import frontmatter
from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams, PointStruct
from google import genai
from dotenv import load_dotenv

# Load env vars from backend/.env
load_dotenv("backend/.env")

# Initialize clients
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", None)
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

if not GEMINI_API_KEY:
    raise ValueError("GEMINI_API_KEY is not set")

qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
client = genai.Client(api_key=GEMINI_API_KEY)

COLLECTION_NAME = "rag_tutor_knowledge_base"

def get_embedding(text):
    # Using text-embedding-004
    response = client.models.embed_content(
        model="text-embedding-004",
        contents=text
    )
    return response.embeddings[0].values

def ingest_docs(docs_dir="website/docs"):
    # Create collection if not exists
    try:
        qdrant.get_collection(COLLECTION_NAME)
    except Exception:
        qdrant.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(size=768, distance=Distance.COSINE), # text-embedding-004 is 768 dims
        )

    files = glob.glob(f"{docs_dir}/**/*.mdx", recursive=True)
    points = []
    
    print(f"Found {len(files)} files to ingest...")

    for i, file_path in enumerate(files):
        with open(file_path, "r") as f:
            try:
                post = frontmatter.load(f)
                content = post.content
                metadata = post.metadata
                
                # Simple chunking by paragraphs for MVP
                chunks = content.split("\n\n")
                for j, chunk in enumerate(chunks):
                    if len(chunk.strip()) < 50: continue # Skip small chunks
                    
                    try:
                        vector = get_embedding(chunk)
                        
                        points.append(PointStruct(
                            id=i * 1000 + j, # Simple ID generation
                            vector=vector,
                            payload={
                                "content": chunk,
                                "source": file_path,
                                "module": metadata.get("id", "unknown"),
                                "title": metadata.get("title", "unknown")
                            }
                        ))
                    except Exception as e:
                        print(f"Error embedding chunk in {file_path}: {e}")

            except Exception as e:
                print(f"Error reading {file_path}: {e}")

    if points:
        qdrant.upsert(
            collection_name=COLLECTION_NAME,
            points=points
        )
        print(f"Ingested {len(points)} chunks from {len(files)} files.")
    else:
        print("No chunks to ingest.")

if __name__ == "__main__":
    ingest_docs()