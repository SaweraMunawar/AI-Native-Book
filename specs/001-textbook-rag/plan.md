# Implementation Plan: AI-Native Textbook with RAG Chatbot

**Branch**: `001-textbook-rag` | **Date**: 2025-12-09 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-textbook-rag/spec.md`

## Summary

Build an AI-native textbook for Physical AI & Humanoid Robotics using Docusaurus for content delivery, with an integrated RAG chatbot powered by Qdrant (vector storage), Neon (metadata), and Groq Llama 3 (response generation). The system operates entirely within free-tier limits and answers questions ONLY from book content.

## Technical Context

**Language/Version**: TypeScript 5.x (Docusaurus frontend), Python 3.11 (RAG backend)
**Primary Dependencies**: Docusaurus 3.x, FastAPI, Qdrant Client, Groq SDK, sentence-transformers
**Storage**: Qdrant Cloud (vectors), Neon PostgreSQL (chat metadata/sessions)
**Testing**: Vitest (frontend), pytest (backend)
**Target Platform**: Web (GitHub Pages static + serverless API)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: <5s chatbot response, <3s page load, <500ms text selection UI
**Constraints**: Free-tier only (Qdrant, Neon, Groq, Vercel/Railway), <100 DAU target
**Scale/Scope**: 6 chapters (~50-100 pages), ~100 DAU, single language primary (English)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Evidence |
|-----------|--------|----------|
| I. Simplicity | ✅ PASS | Minimal stack: Docusaurus + FastAPI + managed services. No custom ML models. |
| II. Accuracy | ✅ PASS | RAG retrieval with citations ensures answers are traceable to source text. |
| III. Minimalism | ✅ PASS | Core features only (P1/P2). Optional features (P3) deferred. No analytics, auth, or tracking. |
| IV. Fast Builds | ✅ PASS | Docusaurus static site with auto-generated sidebar. No heavy preprocessing. |
| V. Free-tier Architecture | ✅ PASS | All services (Qdrant Cloud, Neon, Groq, GitHub Pages) have free tiers. |
| VI. RAG Answers ONLY from Book Text | ✅ PASS | System prompt constrains LLM to retrieved context only. Out-of-scope questions rejected. |

**Gate Result**: PASS - All constitution principles satisfied.

## Project Structure

### Documentation (this feature)

```text
specs/001-textbook-rag/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output (API specs)
└── tasks.md             # Phase 2 output (/sp.tasks)
```

### Source Code (repository root)

```text
# Web application structure (frontend + backend)

docs/                    # Docusaurus content
├── intro.md             # Chapter 1: Introduction to Physical AI
├── humanoid-basics.md   # Chapter 2: Basics of Humanoid Robotics
├── ros2-fundamentals.md # Chapter 3: ROS 2 Fundamentals
├── digital-twin.md      # Chapter 4: Digital Twin Simulation
├── vla-systems.md       # Chapter 5: Vision-Language-Action Systems
└── capstone.md          # Chapter 6: Capstone

website/                 # Docusaurus site
├── docusaurus.config.js
├── sidebars.js          # Auto-generated from docs/
├── src/
│   ├── components/
│   │   ├── ChatBot/     # Floating chatbot UI
│   │   └── SelectToAsk/ # Text selection handler
│   ├── pages/
│   │   └── index.js     # Landing page
│   └── css/
└── static/

backend/                 # RAG API (FastAPI)
├── src/
│   ├── main.py          # FastAPI app entry
│   ├── api/
│   │   ├── chat.py      # /chat endpoint
│   │   └── health.py    # /health endpoint
│   ├── services/
│   │   ├── embeddings.py    # HuggingFace embeddings
│   │   ├── retrieval.py     # Qdrant search
│   │   └── generation.py    # Groq LLM calls
│   ├── models/
│   │   └── schemas.py   # Pydantic models
│   └── config.py        # Environment config
├── scripts/
│   └── ingest.py        # Embed book content to Qdrant
├── tests/
│   ├── unit/
│   └── integration/
└── requirements.txt
```

**Structure Decision**: Web application with separate frontend (Docusaurus static site) and backend (FastAPI serverless). Frontend deploys to GitHub Pages, backend to Vercel/Railway serverless.

## Complexity Tracking

No violations detected. Architecture follows constitution principles with minimal complexity.

## Architecture Decisions

### AD-001: Embedding Model Selection
- **Decision**: Use `sentence-transformers/all-MiniLM-L6-v2` (384 dimensions)
- **Rationale**: Small model size, free HuggingFace inference, good semantic quality for educational content
- **Alternatives Rejected**: OpenAI embeddings (cost), larger models (exceed free tier compute)

### AD-002: Chunk Strategy
- **Decision**: Chunk by paragraph with 512 token max, 50 token overlap
- **Rationale**: Preserves semantic coherence, fits context window, enables precise citations
- **Alternatives Rejected**: Sentence-level (too granular), chapter-level (poor retrieval precision)

### AD-003: Backend Deployment
- **Decision**: Vercel Serverless Functions or Railway free tier
- **Rationale**: Both offer free tier sufficient for 100 DAU, cold start acceptable for <5s target
- **Alternatives Rejected**: Self-hosted (operational burden), AWS Lambda (complexity)

### AD-004: Frontend-Backend Communication
- **Decision**: Direct HTTPS calls from Docusaurus to backend API
- **Rationale**: Simple, no WebSocket complexity, SSE optional for streaming responses
- **Alternatives Rejected**: WebSocket (unnecessary for request-response pattern)
