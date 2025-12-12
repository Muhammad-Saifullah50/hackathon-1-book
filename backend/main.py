from fastapi import FastAPI, Request
from fastapi.responses import Response, StreamingResponse
from fastapi.middleware.cors import CORSMiddleware
from chatkit.server import StreamingResult
from chatkit_server import RagTutorChatKitServer
from dotenv import load_dotenv

# Import the new auth router
from src.api.auth.routes import router as auth_router
# Import the new profile router
from src.api.profile.routes import router as profile_router

load_dotenv()

app = FastAPI()

# Allow CORS for local development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include the auth router
app.include_router(auth_router)
# Include the profile router
app.include_router(profile_router)

# Initialize ChatKit Server
chatkit_server = RagTutorChatKitServer()

@app.get("/")
async def root():
    return {"status": "ok", "service": "RAG Tutor Agent (ChatKit Enabled)"}

@app.post("/chatkit")
async def chatkit_endpoint(request: Request):
    """
    Single endpoint for all ChatKit interactions (session, chat, tools).
    """
    # Process the request using the ChatKit server
    # We pass an empty context dict for now, can be expanded later
    result = await chatkit_server.process(await request.body(), {})
    
    if isinstance(result, StreamingResult):
        return StreamingResponse(result, media_type="text/event-stream")
    else:
        # Return JSON response (for session creation, etc.)
        return Response(content=result.json, media_type="application/json")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)