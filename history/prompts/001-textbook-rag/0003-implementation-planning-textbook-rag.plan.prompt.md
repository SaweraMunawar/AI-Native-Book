---
id: 0003
title: Implementation Planning Textbook RAG
stage: plan
date: 2025-12-09
surface: agent
model: claude-opus-4-5-20251101
feature: 001-textbook-rag
branch: 001-textbook-rag
user: user
command: /sp.plan
labels: ["architecture", "rag", "docusaurus", "fastapi", "planning"]
links:
  spec: specs/001-textbook-rag/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-textbook-rag/plan.md
  - specs/001-textbook-rag/research.md
  - specs/001-textbook-rag/data-model.md
  - specs/001-textbook-rag/contracts/openapi.yaml
  - specs/001-textbook-rag/quickstart.md
  - CLAUDE.md
tests:
  - Constitution check validation (PASSED - all 6 principles)
---

## Prompt

/sp.plan textbook-generation

Create implementation plan for AI-Native Textbook with RAG Chatbot feature.

## Response snapshot

Completed Phase 0 and Phase 1 planning:

**Phase 0 - Research (8 tasks resolved)**:
- RT-001: Docusaurus auto-sidebar configuration
- RT-002: Qdrant Cloud free tier limits (1GB storage - sufficient)
- RT-003: Groq API free tier (14,400 req/day - 14x headroom)
- RT-004: HuggingFace embeddings (local sentence-transformers)
- RT-005: Backend deployment (Railway primary)
- RT-006: Select-to-Ask UI implementation
- RT-007: Confidence threshold (0.7 for high, 0.4 for low)
- RT-008: Neon PostgreSQL usage (metadata only)

**Phase 1 - Design**:
- Data model with 6 entities (Chapter, Section, Embedding, ChatSession, ChatMessage, RateLimit)
- OpenAPI 3.1 contract with /health, /chat, /chat/context endpoints
- Quickstart guide for development setup
- Project structure (frontend/backend split)

**Architecture Decisions**:
- AD-001: MiniLM-L6-v2 embeddings (384 dim)
- AD-002: Paragraph chunking (512 tokens, 50 overlap)
- AD-003: Railway deployment
- AD-004: Direct HTTPS (no WebSocket)

**Constitution Check**: PASS (all 6 principles satisfied)

## Outcome

- ✅ Impact: Complete implementation plan ready for task generation
- 🧪 Tests: Constitution check passed, all 6 principles verified
- 📁 Files: 5 new files created in specs/001-textbook-rag/
- 🔁 Next prompts: /sp.tasks (generate implementation tasks)
- 🧠 Reflection: Research phase resolved all unknowns; free-tier architecture validated

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Constitution gate check - PASS
- Prompt variant (if applicable): Standard /sp.plan flow
- Next experiment (smallest change to try): N/A - proceed to task generation
