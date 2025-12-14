import express from "express";
import cors from "cors";
import dotenv from "dotenv";
import { auth } from "./auth.js";
import { toNodeHandler } from "better-auth/node";

// Load environment variables
dotenv.config();

const app = express();
const PORT = process.env.PORT || 3001;

// Parse CORS origins from environment
const corsOrigins = (process.env.CORS_ORIGINS || "http://localhost:3000,http://localhost:8000").split(",");

// CORS configuration for frontend and backend origins
app.use(
  cors({
    origin: corsOrigins,
    credentials: true,
    methods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allowedHeaders: ["Content-Type", "Authorization"],
  })
);

// Parse JSON bodies
app.use(express.json());

// Health check endpoint
app.get("/health", (req, res) => {
  res.json({
    status: "ok",
    service: "auth-server",
    timestamp: new Date().toISOString(),
  });
});

// Better Auth handler for all /api/auth/* routes
// This handles: sign-up, sign-in, sign-out, session, token, jwks, etc.
app.all("/api/auth/*", toNodeHandler(auth));

// Start server
app.listen(PORT, () => {
  console.log(`Auth server running on http://localhost:${PORT}`);
  console.log(`JWKS endpoint: http://localhost:${PORT}/api/auth/jwks`);
  console.log(`Allowed origins: ${corsOrigins.join(", ")}`);
});

export default app;
