# Quickstart Guide: AI-Native Textbook with RAG Chatbot

**Feature**: 001-textbook-rag
**Date**: 2025-12-09

## Prerequisites

- Node.js 18+ (for Docusaurus)
- Python 3.11+ (for backend)
- Git

## Accounts Required (Free Tier)

1. **Qdrant Cloud**: https://cloud.qdrant.io - Vector database
2. **Neon**: https://neon.tech - PostgreSQL database
3. **Groq**: https://console.groq.com - LLM API
4. **GitHub**: For deployment (GitHub Pages)

## Quick Setup

### 1. Clone and Install

```bash
# Clone repository
git clone https://github.com/your-org/textbook-rag.git
cd textbook-rag

# Install frontend dependencies
cd website
npm install

# Install backend dependencies
cd ../backend
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 2. Environment Configuration

Create `.env` files:

**backend/.env**:
```env
# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# Neon PostgreSQL
DATABASE_URL=postgresql://user:pass@your-neon-host/dbname

# Groq LLM
GROQ_API_KEY=your-groq-api-key

# Application
ENVIRONMENT=development
CORS_ORIGINS=http://localhost:3000
```

**website/.env**:
```env
REACT_APP_API_URL=http://localhost:8000
```

### 3. Database Setup

```bash
# Run Neon migrations
cd backend
python -m scripts.migrate
```

### 4. Ingest Textbook Content

```bash
# Embed textbook content to Qdrant
cd backend
python -m scripts.ingest --docs-path ../docs
```

Expected output:
```
Processing 6 chapters...
  ✓ intro.md: 45 chunks
  ✓ humanoid-basics.md: 62 chunks
  ✓ ros2-fundamentals.md: 78 chunks
  ✓ digital-twin.md: 55 chunks
  ✓ vla-systems.md: 71 chunks
  ✓ capstone.md: 48 chunks
Total: 359 embeddings created
```

### 5. Start Development Servers

**Terminal 1 - Backend**:
```bash
cd backend
source venv/bin/activate
uvicorn src.main:app --reload --port 8000
```

**Terminal 2 - Frontend**:
```bash
cd website
npm start
```

### 6. Verify Setup

1. Open http://localhost:3000
2. Navigate to any chapter
3. Click the chat button (bottom-right)
4. Ask: "What is ROS 2?"
5. Verify response includes citations

## Development Workflow

### Adding/Updating Content

1. Edit markdown files in `docs/`
2. Re-run ingestion: `python -m scripts.ingest --docs-path ../docs`
3. Changes reflect immediately in Docusaurus (hot reload)

### Testing

```bash
# Backend tests
cd backend
pytest tests/ -v

# Frontend tests
cd website
npm test
```

### Building for Production

```bash
# Build frontend
cd website
npm run build

# Output in website/build/ - deploy to GitHub Pages
```

## Deployment

### GitHub Pages (Frontend)

```yaml
# .github/workflows/deploy.yml
name: Deploy to GitHub Pages
on:
  push:
    branches: [main]
jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 18
      - run: cd website && npm ci && npm run build
      - uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./website/build
```

### Railway (Backend)

1. Connect GitHub repo to Railway
2. Set environment variables in Railway dashboard
3. Deploy triggers on push to main

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Qdrant connection failed | Check QDRANT_URL and API key |
| Slow chatbot responses | Verify Groq API key, check rate limits |
| Missing citations | Re-run ingestion script |
| CORS errors | Add frontend URL to CORS_ORIGINS |

## Next Steps

- Run `/sp.tasks` to generate implementation tasks
- Review `contracts/openapi.yaml` for API details
- Check `data-model.md` for entity specifications
