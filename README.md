# Physical AI & Humanoid Robotics Learning Platform

A world-class technical textbook platform bridging digital AI with embodied intelligence. This monorepo hosts a full-stack educational platform combining a rigorous academic curriculum with an interactive AI tutor.

![Status](https://img.shields.io/badge/status-active-success.svg)
![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)

---

## 🚀 Overview

This project is a **hybrid full-stack monorepo** designed to teach Physical AI and Humanoid Robotics. It goes beyond static content by integrating a RAG-powered AI tutor that helps learners understand complex concepts like kinematics, SLAM, and VLA (Vision-Language-Action) models.

### Key Features
- 📚 **Interactive Textbook**: Built with Docusaurus, featuring 4 modules and 13 weeks of content.
- 🤖 **RAG Tutor Agent**: specialized AI tutor powered by OpenAI and Qdrant to answer questions based on the book's content.
- 🔐 **Authentication & Profiles**: Secure user accounts with Better Auth, enabling personalized learning paths.
- 🛠️ **Sim-to-Real Focus**: Emphasizes the transition from simulation (Digital Twins) to real-world deployment.
- 🌑 **Dark Mode**: Fully accessible UI with dark mode support.

---

## 🏗️ Architecture

The platform consists of three main services working together:

```mermaid
graph TD
    Client[Browser / Docusaurus]
    Auth[Auth Server (Node.js/Better Auth)]
    API[Backend API (FastAPI)]
    DB[(Neon PostgreSQL)]
    Vector[(Qdrant Vector DB)]
    AI[OpenAI API]

    Client -->|Auth Requests| Auth
    Client -->|RAG / Profile Data| API
    Auth -->|Session/User Data| DB
    API -->|Verify Token| Auth
    API -->|Read/Write Profile| DB
    API -->|Retrieve Context| Vector
    API -->|Generate Answers| AI
```

---

## 🛠️ Technology Stack

### Frontend (`website/`)
- **Framework**: Docusaurus 3.9 (React 19)
- **Language**: TypeScript 5.6
- **Styling**: Tailwind CSS 3.4
- **Components**: Radix UI, Framer Motion
- **State/Hooks**: Custom React hooks (`useAuth`, `useProfile`)
- **Testing**: Jest, Playwright

### Backend (`backend/`)
- **Framework**: FastAPI (Python 3.12)
- **Server**: Uvicorn
- **AI/RAG**: `openai-agents`, `qdrant-client`
- **Validation**: Pydantic
- **Testing**: PyTest

### Auth Server (`auth-server/`)
- **Framework**: Express (Node.js >= 20)
- **Auth Engine**: Better Auth
- **Database**: Neon PostgreSQL

---

## 🏁 Getting Started

### Prerequisites
- **Node.js**: >= 20.0
- **Python**: >= 3.12
- **uv**: Python package manager (`pip install uv`)
- **Git**

### 1. Clone the Repository
```bash
git clone https://github.com/Muhammad-Saifullah50/hackathon-1-book.git
cd hackathon-1-book
```

### 2. Environment Setup
You will need `.env` files for each service.

**Backend (`backend/.env`):**
```env
OPENAI_API_KEY=sk-...
QDRANT_URL=http://localhost:6333
# Database URL (Neon)
DATABASE_URL=postgresql://user:pass@host/db
```

**Auth Server (`auth-server/.env`):**
```env
PORT=3001
DATABASE_URL=postgresql://user:pass@host/db
BETTER_AUTH_SECRET=your-32-char-secret
BETTER_AUTH_URL=http://localhost:3001
CORS_ORIGINS=http://localhost:3000,http://localhost:8000
```

### 3. Install Dependencies & Run

**Terminal 1: Auth Server**
```bash
cd auth-server
npm install
npm run dev
# Runs on http://localhost:3001
```

**Terminal 2: Backend API**
```bash
cd backend
uv sync
uv run uvicorn main:app --reload
# Runs on http://localhost:8000
```

**Terminal 3: Frontend Website**
```bash
cd website
npm install
npm start
# Runs on http://localhost:3000
```

---

## 📂 Project Structure

```
/
├── website/               # Docusaurus frontend
│   ├── src/               # React components, pages, hooks
│   ├── docs/              # Markdown curriculum content
│   └── tests/             # Frontend tests
├── backend/               # FastAPI backend
│   ├── src/               # API routes, services, models
│   └── tests/             # Backend tests
├── auth-server/           # Better Auth node server
├── specs/                 # SDD Specifications (Feature specs)
├── scripts/               # Utility scripts (Ingestion, Migrations)
└── history/               # Project history (PHRs, ADRs)
```

---

## 🧪 Development & Testing

This project follows **Spec-Driven Development (SDD)** and **Test-Driven Development (TDD)**.

### Running Tests
- **Frontend Unit**: `cd website && npm test`
- **Frontend E2E**: `npm run test:e2e`
- **Backend**: `cd backend && uv run pytest`

### Creating New Features
1.  **Specify**: Create a spec in `specs/<feature-id>/spec.md`.
2.  **Plan**: Generate a plan and tasks.
3.  **Test**: Write failing tests (Red).
4.  **Implement**: Write code to pass tests (Green).
5.  **Refactor**: Optimize and clean up.

---

## 📄 License

This project is licensed under the MIT License.
