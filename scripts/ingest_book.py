import os
import glob
import time
import frontmatter
from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams, PointStruct
from google import genai
from dotenv import load_dotenv

# Load env vars from backend/.env
script_dir = os.path.dirname(os.path.abspath(__file__))
backend_env = os.path.join(script_dir, "..", "backend", ".env")
load_dotenv(backend_env, override=True)

# Initialize clients
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", None)
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

if not GEMINI_API_KEY:
    raise ValueError("GEMINI_API_KEY is not set")

qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
client = genai.Client(api_key=GEMINI_API_KEY)

COLLECTION_NAME = "rag_tutor_knowledge_base"


def get_embedding(text, retries=5):
    # Using gemini-embedding-001 (3072 dims)
    for attempt in range(retries):
        try:
            response = client.models.embed_content(
                model="gemini-embedding-001", contents=text
            )
            return response.embeddings[0].values
        except Exception as e:
            if "RESOURCE_EXHAUSTED" in str(e) or "429" in str(e):
                wait = 70 * (attempt + 1)
                print(
                    f"Rate limited. Waiting {wait}s before retry (attempt {attempt + 1}/{retries})..."
                )
                time.sleep(wait)
            else:
                raise
    print(f"Failed to embed after {retries} retries")
    return None


def ingest_docs(docs_dir="website/docs"):
    # Verify embedding model works
    print(f"Embedding model: gemini-embedding-001")
    try:
        test = client.models.embed_content(
            model="gemini-embedding-001", contents="test"
        )
        dims = len(test.embeddings[0].values)
        print(f"Embedding model OK — vector dims: {dims}")
    except Exception as e:
        print(f"ERROR: Embedding model verification failed: {e}")
        raise

    # Create collection if not exists
    try:
        existing = qdrant.get_collection(COLLECTION_NAME)
        old_count = existing.points_count
        print(f"Collection '{COLLECTION_NAME}' exists — {old_count} existing points")
    except Exception:
        print(
            f"Creating collection '{COLLECTION_NAME}' with size={dims}, COSINE distance"
        )
        qdrant.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(size=dims, distance=Distance.COSINE),
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
                    if len(chunk.strip()) < 50:
                        continue  # Skip small chunks

                    try:
                        vector = get_embedding(chunk)
                        if vector is None:
                            continue

                        points.append(
                            PointStruct(
                                id=i * 1000 + j,  # Simple ID generation
                                vector=vector,
                                payload={
                                    "content": chunk,
                                    "source": file_path,
                                    "module": metadata.get("id", "unknown"),
                                    "title": metadata.get("title", "unknown"),
                                },
                            )
                        )
                        print(
                            f"  Chunk {j + 1} embedded for {os.path.basename(file_path)}"
                        )
                    except Exception as e:
                        print(f"Error embedding chunk in {file_path}: {e}")

            except Exception as e:
                print(f"Error reading {file_path}: {e}")

    if points:
        qdrant.upsert(collection_name=COLLECTION_NAME, points=points)
        info = qdrant.get_collection(COLLECTION_NAME)
        print(f"Ingested {len(points)} chunks from {len(files)} files.")
        print(f"Collection now has {info.points_count} total points.")
    else:
        print("No chunks to ingest.")


if __name__ == "__main__":
    ingest_docs()
