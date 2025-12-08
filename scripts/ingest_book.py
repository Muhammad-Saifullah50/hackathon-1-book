import os
import glob
import frontmatter
from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams, PointStruct
from openai import OpenAI

# Initialize clients
qdrant = QdrantClient(url=os.getenv("QDRANT_URL", "http://localhost:6333"))
openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

COLLECTION_NAME = "rag_tutor_knowledge_base"

def get_embedding(text):
    response = openai_client.embeddings.create(
        input=text,
        model="text-embedding-3-small"
    )
    return response.data[0].embedding

def ingest_docs(docs_dir="website/docs"):
    # Create collection if not exists
    try:
        qdrant.get_collection(COLLECTION_NAME)
    except Exception:
        qdrant.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(size=1536, distance=Distance.COSINE),
        )

    files = glob.glob(f"{docs_dir}/**/*.mdx", recursive=True)
    points = []
    
    for i, file_path in enumerate(files):
        with open(file_path, "r") as f:
            post = frontmatter.load(f)
            content = post.content
            metadata = post.metadata
            
            # Simple chunking by paragraphs for MVP
            chunks = content.split("\n\n")
            for j, chunk in enumerate(chunks):
                if len(chunk.strip()) < 50: continue # Skip small chunks
                
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

    if points:
        qdrant.upsert(
            collection_name=COLLECTION_NAME,
            points=points
        )
        print(f"Ingested {len(points)} chunks from {len(files)} files.")

if __name__ == "__main__":
    ingest_docs()
