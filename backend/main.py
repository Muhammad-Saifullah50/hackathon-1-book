from pathlib import Path
from typing import Optional
from fastapi import FastAPI, Request, Depends
from fastapi.responses import Response, StreamingResponse
from fastapi.middleware.cors import CORSMiddleware
from chatkit.server import StreamingResult
from chatkit_server import RagTutorChatKitServer
from dotenv import load_dotenv

# Import routers
from src.api.profile.routes import router as profile_router
from src.api.personalization.routes import router as personalization_router
from src.api.translation.routes import router as translation_router
from src.api.auth.dependencies import CurrentUser, get_optional_user

# Load .env from the backend directory (where this file is located)
env_path = Path(__file__).parent / ".env"
load_dotenv(env_path)

app = FastAPI()

# Allow CORS for local development
# Includes: Docusaurus frontend, Auth server, and production origins
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://robotook.vercel.app",  # Docusaurus dev server
        "https://robotook-auth.vercel.app",  # Auth server
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(profile_router)
app.include_router(personalization_router)
app.include_router(translation_router)

# Initialize ChatKit Server
chatkit_server = RagTutorChatKitServer()

@app.get("/")
async def root():
    return {"status": "ok", "service": "RAG Tutor Agent (ChatKit Enabled)"}


@app.get("/health")
async def health_check():
    """
    Health check endpoint with database connectivity status.

    Returns:
        JSON with status, service name, and database connection status
    """
    from src.services.database import check_database_connection

    db_connected = check_database_connection()

    return {
        "status": "ok",
        "service": "Physical AI & Robotics Platform",
        "database": "connected" if db_connected else "disconnected",
    }

@app.post("/chatkit")
async def chatkit_endpoint(request: Request, user: Optional[CurrentUser] = Depends(get_optional_user)):
    """
    Single endpoint for all ChatKit interactions (session, chat, tools).
    Supports both authenticated and anonymous users.
    """
    # Build context with user_id for authenticated users
    # Anonymous users get empty context (in-memory session storage only)
    context = {}
    if user:
        context["user_id"] = user.id

    # Process the request using the ChatKit server
    result = await chatkit_server.process(await request.body(), context)

    if isinstance(result, StreamingResult):
        return StreamingResponse(result, media_type="text/event-stream")
    else:
        # Return JSON response (for session creation, etc.)
        return Response(content=result.json, media_type="application/json")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)