# Physical AI & Humanoid Robotics Learning Platform

<div align="center">

**A Technical Textbook Platform Bridging Digital AI with Embodied Intelligence**

[![Status](https://img.shields.io/badge/status-active-success.svg)](https://github.com/Muhammad-Saifullah50/hackathon-1-book)
[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)](https://github.com/Muhammad-Saifullah50/hackathon-1-book)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![Node](https://img.shields.io/badge/node-%3E%3D20.0-brightgreen.svg)](https://nodejs.org)
[![Python](https://img.shields.io/badge/python-%3E%3D3.12-blue.svg)](https://python.org)

[Features](#-features) • [Architecture](#-architecture) • [Getting Started](#-getting-started) • [Documentation](#-documentation) • [Contributing](#-contributing)

</div>

---

## Overview

This project is a **hybrid full-stack monorepo** that delivers an interactive educational experience for Physical AI and Humanoid Robotics. Going beyond traditional static textbooks, it combines rigorous academic curriculum with an intelligent RAG-powered AI tutor that understands complex concepts like kinematics, SLAM (Simultaneous Localization and Mapping), and VLA (Vision-Language-Action) models.

The platform emphasizes a **Sim-to-Real** workflow: learners design algorithms, test them in digital twin simulations, and understand the considerations for real-world deployment—complete with physics constraints like gravity, friction, and sensor noise.

### What Makes This Special

- **Comprehensive Curriculum**: 4 modules spanning 13 weeks, from foundations to cutting-edge VLA models
- **Intelligent Tutoring**: Context-aware AI assistant trained on the entire curriculum using RAG (Retrieval Augmented Generation)
- **Personalized Learning**: Adaptive content based on learner background, goals, and pace
- **Production-Grade Stack**: Modern TypeScript/React frontend + Python/FastAPI backend + Neon PostgreSQL
- **Spec-Driven Development**: Every feature backed by formal specifications and ADRs (Architecture Decision Records)

---

## Features

### Core Learning Experience
- **Interactive Textbook**: Built with Docusaurus 3.9, featuring rich markdown content, code snippets, and embedded visualizations
- **4 Comprehensive Modules**:
  - Module 1: Foundations (Robotics history, kinematics, control systems)
  - Module 2: Digital Twin & Simulation (Physics engines, sensor models, Sim-to-Real gap)
  - Module 3: AI-Robot Brain (Computer vision, NLP, reinforcement learning)
  - Module 4: VLA Models (Vision-Language-Action integration, embodied AI)

### Intelligent AI Tutor
- **RAG-Powered Chatbot**: Ask questions and get answers grounded in the textbook content
- **Context-Aware Responses**: Retrieves relevant sections from Qdrant vector database
- **Persistent Conversations**: Chat history stored per user for continuity
- **Multi-Modal Understanding**: Handles questions about code, diagrams, and theoretical concepts

### Personalization & Accessibility
- **Learning Profile Wizard**: Customizes content based on:
  - Education level and technical background
  - Learning goals (career, research, hobby, education)
  - Preferred learning mode (visual, reading, hands-on, mixed)
  - Time commitment per week
- **Multi-Language Support**: English, Spanish, Chinese, Arabic translation (Urdu in progress)
- **Dark Mode**: Fully accessible UI with theme switching
- **Progress Tracking**: Monitor completion across modules and weeks

### Authentication & Security
- **Secure Authentication**: Better Auth integration with JWT tokens
- **User Profiles**: Persistent learner profiles and preferences
- **Session Management**: Secure token refresh and logout flows
- **Privacy-First**: Data stored in Neon serverless PostgreSQL with connection pooling

---

## Architecture

The platform consists of three main services working together in a monorepo:

```
┌─────────────────┐
│  Browser Client │
│  Docusaurus 3.9 │
│  React 19       │
└────────┬────────┘
         │
         ├──── Auth Requests ────────────────┐
         │                                   │
         └──── RAG / Profile Data ───┐      │
                                      ▼      ▼
                             ┌─────────────────┐      ┌──────────────┐
                             │  Backend API    │◄────►│ Auth Server  │
                             │  FastAPI        │      │ Better Auth  │
                             │  Python 3.12    │      │ Express      │
                             └────────┬────────┘      └──────┬───────┘
                                      │                      │
                         ┌────────────┼──────────────────────┘
                         │            │
                         ▼            ▼
                  ┌─────────┐  ┌─────────────┐
                  │ Qdrant  │  │    Neon     │
                  │ Vector  │  │ PostgreSQL  │
                  │   DB    │  │ (Serverless)│
                  └────┬────┘  └─────────────┘
                       │
                       │ Embeddings
                       ▼
                  ┌─────────┐
                  │ OpenAI  │
                  │  API    │
                  │ GPT-4   │
                  └─────────┘
```

### Service Details

| Service | Technology | Port | Purpose |
|---------|-----------|------|---------|
| **Frontend** | Docusaurus 3.9 + React 19 | 3000 | Interactive textbook UI |
| **Auth Server** | Better Auth + Express | 3001 | User authentication |
| **Backend API** | FastAPI + Python 3.12 | 8000 | RAG tutor + profile management |
| **Database** | Neon PostgreSQL | - | User data, profiles, chat history |
| **Vector DB** | Qdrant | 6333 | Embedded textbook content for RAG |

---

## Technology Stack

### Frontend (`website/`)

| Technology | Version | Purpose |
|------------|---------|---------|
| **Docusaurus** | 3.9.2 | Static site generator with React |
| **React** | 19.0.0 | UI framework with modern hooks |
| **TypeScript** | 5.6.2 | Type-safe JavaScript |
| **Tailwind CSS** | 3.4.18 | Utility-first styling |
| **Radix UI** | 2.2.6+ | Accessible component primitives |
| **Framer Motion** | 12.23.26 | Smooth animations |
| **Zod** | 4.1.13 | Schema validation |
| **Better Auth** | 1.2.8 | Authentication client |
| **OpenAI ChatKit** | 1.3.0 | Chat UI components |
| **Jest** | 30.2.0 | Unit testing |
| **Playwright** | 1.57.0 | E2E testing |

### Backend (`backend/`)

| Technology | Version | Purpose |
|------------|---------|---------|
| **FastAPI** | Latest | Async Python web framework |
| **Python** | 3.12+ | Runtime |
| **uvicorn** | Latest | ASGI server |
| **openai-agents** | 0.6.2+ | OpenAI agent framework |
| **openai-chatkit** | 1.4.0+ | Chat backend utilities |
| **qdrant-client** | Latest | Vector database client |
| **asyncpg** | 0.31.0+ | Async PostgreSQL driver |
| **Pydantic** | 2.12.5+ | Data validation |
| **pytest** | Latest | Testing framework |

### Auth Server (`auth-server/`)

| Technology | Version | Purpose |
|------------|---------|---------|
| **Better Auth** | 1.2.8 | Modern auth framework |
| **Express** | 4.21.2 | Node.js web server |
| **TypeScript** | 5.7.2 | Type-safe backend |
| **pg** | 8.13.1 | PostgreSQL client |

### Infrastructure

| Component | Technology | Purpose |
|-----------|-----------|---------|
| **Database** | Neon PostgreSQL (Serverless) | User data, profiles, sessions |
| **Vector DB** | Qdrant | Semantic search for RAG |
| **AI Provider** | OpenAI (GPT-4 + Embeddings) | Chat generation + embeddings |
| **Package Managers** | npm (frontend/auth) + uv (backend) | Dependency management |

---

## Project Structure

```
hackathon-1-book/
├── website/                    # Docusaurus frontend application
│   ├── src/
│   │   ├── components/         # React components
│   │   │   ├── auth/           # Login, signup, auth navbar
│   │   │   ├── profile/        # Profile wizard, personalization
│   │   │   ├── landing/        # Hero, features, demos
│   │   │   ├── ui/             # Reusable UI components (Radix)
│   │   │   ├── translation/    # Translation bar
│   │   │   └── ChatWidget.tsx  # RAG chatbot interface
│   │   ├── hooks/              # Custom React hooks
│   │   │   ├── useAuth.tsx     # Authentication state
│   │   │   ├── useProfile.tsx  # User profile management
│   │   │   ├── usePersonalization.ts
│   │   │   └── useTranslation.ts
│   │   ├── pages/              # Custom pages
│   │   │   ├── signup.tsx
│   │   │   ├── login.tsx
│   │   │   └── signup-wizard.tsx
│   │   ├── types/              # TypeScript interfaces
│   │   └── theme/              # Docusaurus theme overrides
│   ├── docs/                   # Markdown curriculum content
│   │   ├── module-01/          # Foundations
│   │   ├── module-02/          # Digital Twin
│   │   ├── module-03/          # AI-Robot Brain
│   │   └── module-04/          # VLA Models
│   ├── static/                 # Static assets (images, diagrams)
│   ├── tests/                  # Frontend tests
│   ├── docusaurus.config.ts    # Site configuration
│   ├── tailwind.config.js      # Tailwind theme
│   └── package.json            # Dependencies
│
├── backend/                    # Python FastAPI backend
│   ├── src/
│   │   ├── api/                # API route handlers
│   │   ├── services/           # Business logic layer
│   │   │   ├── auth.py
│   │   │   ├── profile.py
│   │   │   └── chat_store.py   # Chat history management
│   │   └── models/             # Pydantic data models
│   ├── tests/                  # PyTest tests
│   ├── main.py                 # FastAPI app entry point
│   ├── agent_tools.py          # RAG agent configuration
│   └── pyproject.tomllll       # Python dependencies (uv)
│
├── auth-server/                # Better Auth authentication server
│   ├── src/
│   │   ├── index.ts            # Express server
│   │   ├── auth.ts             # Better Auth configuration
│   │   └── db.ts               # Database connection
│   ├── migrations/             # Database migrations
│   └── package.json            # Dependencies
│
├── scripts/                    # Utility scripts
│   └── ingest_book.py          # Qdrant content ingestion
│
├── specs/                      # SDD feature specifications
│   ├── 001-core-learning/
│   ├── 007-rag-tutor/
│   ├── 008-auth-profile/
│   ├── 010-neon-migration/
│   └── ...
│
├── history/                    # Project history
│   ├── prompts/                # PHRs (Prompt History Records)
│   └── decisions/              # ADRs (Architecture Decision Records)
│
├── tests/                      # Root-level E2E tests
│   └── e2e/                    # Cross-service integration tests
│
├── .specify/                   # SpecKit Plus templates
│   ├── memory/
│   │   └── constitution.md     # Project principles
│   └── templates/              # SDD templates
│
├── CLAUDE.md                   # AI assistant instructions
├── README.md                   # This file
└── .gitignore
```

---

## Getting Started

### Prerequisites

Ensure you have the following installed:

- **Node.js**: >= 20.0 ([Download](https://nodejs.org))
- **Python**: >= 3.12 ([Download](https://python.org))
- **uv**: Python package manager (`pip install uv`)
- **Git**: For version control
- **Qdrant**: Vector database (Docker recommended)

### 1. Clone the Repository

```bash
git clone https://github.com/Muhammad-Saifullah50/hackathon-1-book.git
cd hackathon-1-book
```

### 2. Set Up Qdrant Vector Database

Using Docker (recommended):

```bash
docker run -p 6333:6333 qdrant/qdrant
```

Or follow the [Qdrant installation guide](https://qdrant.tech/documentation/quick-start/).

### 3. Configure Environment Variables

You'll need `.env` files for each service. Use the templates below:

#### Backend (`backend/.env`)

```env
# OpenAI API Key (required for RAG)
OPENAI_API_KEY=sk-...

# Qdrant Vector Database
QDRANT_URL=http://localhost:6333

# Neon PostgreSQL Database
DATABASE_URL=postgresql://user:password@host:5432/database?sslmode=require

# Auth Server URL
AUTH_SERVER_URL=http://localhost:3001
```

#### Auth Server (`auth-server/.env`)

```env
# Server Configuration
PORT=3001
NODE_ENV=development

# Database (Neon PostgreSQL)
DATABASE_URL=postgresql://user:password@host:5432/database?sslmode=require

# Better Auth Configuration
BETTER_AUTH_SECRET=your-random-32-character-secret-key-here
BETTER_AUTH_URL=http://localhost:3001

# CORS Configuration
CORS_ORIGINS=http://localhost:3000,http://localhost:8000

# Trust Proxy (for production deployments)
TRUST_PROXY=false
```

#### Frontend (`website/.env` - optional)

The frontend reads backend URL from `docusaurus.config.ts`. You can override with:

```env
BACKEND_URL=http://localhost:8000
AUTH_SERVER_URL=http://localhost:3001
```

### 4. Install Dependencies & Run Services

You'll need **three terminal windows** running simultaneously:

#### Terminal 1: Auth Server

```bash
cd auth-server
npm install
npm run dev
```

Server starts on `http://localhost:3001`

#### Terminal 2: Backend API

```bash
cd backend
uv sync                                    # Install dependencies
uv run uvicorn main:app --reload          # Start server
```

Server starts on `http://localhost:8000`

Optional: Ingest textbook content to Qdrant:

```bash
uv run python scripts/ingest_book.py
```

#### Terminal 3: Frontend Website

```bash
cd website
npm install
npm start
```

Server starts on `http://localhost:3000`

### 5. Verify Installation

Open your browser and navigate to:

- **Frontend**: [http://localhost:3000](http://localhost:3000)
- **Backend API Docs**: [http://localhost:8000/docs](http://localhost:8000/docs)
- **Auth Server Health**: [http://localhost:3001/health](http://localhost:3001/health)

You should see the landing page with options to sign up, log in, and explore the curriculum.

---

## Development Workflow

This project follows **Spec-Driven Development (SDD)** and **Test-Driven Development (TDD)**.

### Creating a New Feature

1. **Specify**: Create a formal specification in `specs/<feature-id>/spec.md`
2. **Plan**: Generate implementation plan and tasks
3. **Test (Red)**: Write failing tests that define the feature
4. **Implement (Green)**: Write code to make tests pass
5. **Refactor**: Optimize and clean up while keeping tests green
6. **Document**: Create PHR (Prompt History Record) and ADR if needed

### Running Tests

#### Frontend Tests

```bash
cd website

# Unit tests with Jest
npm test

# Watch mode for TDD
npm run test:watch

# E2E tests with Playwright
npm run test:e2e
```

#### Backend Tests

```bash
cd backend

# Run all tests
uv run pytest

# Run with coverage
uv run pytest --cov

# Run specific test file
uv run pytest tests/test_auth.py
```

### Code Quality

**Frontend (TypeScript):**
- Use functional components with React hooks
- Type all props and state with TypeScript interfaces
- Validate data with Zod schemas
- Follow Tailwind CSS utility-first patterns
- Use Radix UI for accessible components
- Prefer `useSafeColorMode` over `useColorMode` for theme

**Backend (Python):**
- Use full async/await patterns
- Define Pydantic models for all data
- Implement service layer for business logic
- Use dependency injection with `Depends`
- Raise appropriate HTTPExceptions
- Document all API endpoints with docstrings

### Git Workflow

```bash
# Create feature branch
git checkout -b feature/your-feature-name

# Make changes and commit
git add .
git commit -m "feat: add your feature description"

# Push and create PR
git push origin feature/your-feature-name
```

Commit message convention: `type: description`
- `feat:` New feature
- `fix:` Bug fix
- `docs:` Documentation
- `test:` Tests
- `refactor:` Code refactoring
- `chore:` Maintenance

---

## Documentation

### Component Documentation

- **Frontend Components**: See `website/src/components/README.md` (if available)
- **Backend API**: Interactive docs at `http://localhost:8000/docs` (FastAPI Swagger UI)
- **Auth API**: Docs at `http://localhost:3001/api/docs` (Better Auth endpoints)

### Key Concepts

#### RAG (Retrieval Augmented Generation)
The AI tutor uses RAG to ground responses in textbook content:
1. User asks a question
2. Question is embedded using OpenAI embeddings
3. Qdrant finds semantically similar textbook sections
4. Retrieved context is passed to GPT-4
5. GPT-4 generates an answer based on the context

#### Personalization
User profiles customize the learning experience:
- **Age Range**: Adjusts complexity of explanations
- **Education Level**: Tailors prerequisite assumptions
- **Tech Background**: Determines starting point
- **Learning Mode**: Influences content presentation (visual, text, hands-on)
- **Learning Speed**: Affects pacing suggestions

#### Sim-to-Real Workflow
The curriculum emphasizes:
1. **Design**: Mathematical models and algorithms
2. **Simulate**: Test in digital twin environments (physics engines)
3. **Reality Gap**: Understand limitations (sensor noise, model mismatch)
4. **Deploy**: Adapt for real hardware with safety protocols

### Architecture Decision Records

Important architectural decisions are documented in `history/decisions/`:
- `001-neon-migration.md`: Why we migrated from Supabase to Neon
- `002-better-auth.md`: Why we chose Better Auth over Supabase Auth
- `003-rag-architecture.md`: RAG tutor design decisions

---

## Deployment

### Production Considerations

**Environment Variables**: Never commit `.env` files. Use platform-specific secrets management.

**Database**: Neon PostgreSQL is serverless and scales automatically. Connection pooling is enabled via `asyncpg`.

**Auth Server**:
- Set `TRUST_PROXY=true` in production
- Use HTTPS for `BETTER_AUTH_URL`
- Update `CORS_ORIGINS` to production domains
- Set secure cookie settings

**Backend API**:
- Use production ASGI server (uvicorn workers or gunicorn)
- Enable CORS for production frontend domain
- Set rate limiting on API endpoints
- Monitor Qdrant vector DB performance

**Frontend**:
- Build with `npm run build` in `website/`
- Serve static files with CDN (Vercel, Netlify, Cloudflare Pages)
- Update `docusaurus.config.ts` with production URLs

### Recommended Platforms

- **Frontend**: Vercel, Netlify, Cloudflare Pages
- **Backend**: Railway, Render, Fly.io
- **Auth Server**: Railway, Render
- **Database**: Neon (already serverless)
- **Vector DB**: Qdrant Cloud, self-hosted

---

## Contributing

We welcome contributions to improve the learning platform and curriculum!

### How to Contribute

1. **Fork the Repository**: Click "Fork" on GitHub
2. **Clone Your Fork**: `git clone https://github.com/YOUR_USERNAME/hackathon-1-book.git`
3. **Create a Branch**: `git checkout -b feature/your-feature-name`
4. **Make Changes**: Follow code quality guidelines
5. **Write Tests**: Ensure all tests pass
6. **Commit**: Use conventional commit messages
7. **Push**: `git push origin feature/your-feature-name`
8. **Open Pull Request**: Describe your changes clearly

### Contribution Guidelines

- **Code Quality**: Follow existing patterns and style guides
- **Testing**: All new features must include tests (TDD)
- **Documentation**: Update README or docs if needed
- **Small PRs**: Keep pull requests focused and small
- **Commit Messages**: Use conventional commits (`feat:`, `fix:`, `docs:`, etc.)

### Areas We Need Help

- **Content Creation**: Writing new curriculum sections
- **Translation**: Translating content to more languages
- **Accessibility**: Improving WCAG compliance
- **Performance**: Optimizing load times and bundle sizes
- **Bug Fixes**: Addressing issues in the tracker
- **Testing**: Expanding test coverage

### Code of Conduct

Be respectful, inclusive, and constructive. We're building an educational platform—let's maintain a welcoming learning environment for everyone.

---

## Troubleshooting

### Common Issues

**1. Auth server won't start**
```bash
Error: connect ECONNREFUSED localhost:5432
```
**Solution**: Check `DATABASE_URL` in `auth-server/.env`. Ensure Neon PostgreSQL is accessible.

**2. Backend can't connect to Qdrant**
```bash
Error: cannot connect to qdrant at http://localhost:6333
```
**Solution**: Start Qdrant using Docker: `docker run -p 6333:6333 qdrant/qdrant`

**3. Frontend shows "Unauthorized"**
**Solution**:
- Ensure auth server is running on port 3001
- Check `AUTH_SERVER_URL` in frontend configuration
- Clear browser cookies and retry login

**4. RAG chatbot returns empty responses**
**Solution**: Ingest textbook content to Qdrant:
```bash
cd backend
uv run python scripts/ingest_book.py
```

**5. Port already in use**
```bash
Error: Port 3000 is already in use
```
**Solution**: Kill the process using the port:
```bash
# Find process
lsof -i :3000

# Kill process
kill -9 <PID>
```

### Getting Help

- **Issues**: Open an issue on [GitHub Issues](https://github.com/Muhammad-Saifullah50/hackathon-1-book/issues)
- **Discussions**: Join [GitHub Discussions](https://github.com/Muhammad-Saifullah50/hackathon-1-book/discussions)
- **Email**: Contact maintainers (see below)

---

## Roadmap

### Planned Features

- [ ] **Lab Environment**: Interactive Jupyter notebooks for hands-on coding
- [ ] **Video Integration**: Embedded tutorial videos and demonstrations
- [ ] **Code Sandbox**: In-browser Python/simulation environment
- [ ] **Progress Dashboard**: Detailed analytics and completion tracking
- [ ] **Community Forum**: Peer-to-peer learning and discussion
- [ ] **Mobile App**: Native iOS/Android companion apps
- [ ] **Certification**: Issue blockchain-verified completion certificates
- [ ] **Hardware Kits**: Integrate with physical robotics kits (Arduino, Raspberry Pi)

### Current Focus

- [x] Core curriculum (Modules 1-4) ✅
- [x] RAG-powered AI tutor ✅
- [x] User authentication and profiles ✅
- [x] Personalization engine ✅
- [x] Dark mode and accessibility ✅
- [x] Multi-language support (partial) ✅
- [ ] Lab environment (Q1 2025)
- [ ] Mobile apps (Q2 2025)

---

## License

This project is licensed under the **MIT License**. See [LICENSE](LICENSE) file for details.

You are free to:
- Use the platform for personal or commercial purposes
- Modify and distribute the code
- Include it in proprietary software

**Attribution**: Please credit "Physical AI & Humanoid Robotics Learning Platform" when redistributing.

---

## Acknowledgments

### Built With

- [Docusaurus](https://docusaurus.io) - Static site generator
- [FastAPI](https://fastapi.tiangolo.com) - Python web framework
- [Better Auth](https://www.better-auth.com) - Modern authentication
- [OpenAI](https://openai.com) - GPT-4 and embeddings
- [Qdrant](https://qdrant.tech) - Vector database for RAG
- [Neon](https://neon.tech) - Serverless PostgreSQL
- [Radix UI](https://www.radix-ui.com) - Accessible components
- [Tailwind CSS](https://tailwindcss.com) - Utility-first CSS

### Special Thanks

- **OpenAI** for GPT-4 and embedding models
- **Anthropic** for Claude AI (used in development workflow)
- **Robotics Community** for inspiration and feedback
- **Open Source Contributors** who make projects like this possible

---

## Contact

**Project Maintainer**: Muhammad Saifullah
**GitHub**: [@Muhammad-Saifullah50](https://github.com/Muhammad-Saifullah50)
**Repository**: [hackathon-1-book](https://github.com/Muhammad-Saifullah50/hackathon-1-book)

For bugs and feature requests, please use [GitHub Issues](https://github.com/Muhammad-Saifullah50/hackathon-1-book/issues).

---

<div align="center">

**Built with passion for robotics education and embodied AI**

⭐ Star this repo if you find it helpful! ⭐

[Report Bug](https://github.com/Muhammad-Saifullah50/hackathon-1-book/issues) · [Request Feature](https://github.com/Muhammad-Saifullah50/hackathon-1-book/issues) · [Contribute](CONTRIBUTING.md)

</div>
