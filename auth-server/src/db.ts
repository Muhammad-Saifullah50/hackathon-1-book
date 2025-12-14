import { Pool } from "pg";
import dotenv from "dotenv";

dotenv.config();

/**
 * Neon PostgreSQL connection pool for Better Auth
 * Uses connection string with SSL required for Neon
 */
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: {
    rejectUnauthorized: false, // Required for Neon serverless
  },
  max: 10, // Maximum connections in pool
  idleTimeoutMillis: 30000, // Close idle connections after 30s
  connectionTimeoutMillis: 10000, // 10s connection timeout
});

// Test connection on startup
pool.on("connect", () => {
  console.log("Connected to Neon PostgreSQL database");
});

pool.on("error", (err) => {
  console.error("Unexpected error on idle client", err);
});

export { pool };
export default pool;
