# Physical AI & Humanoid Robotics Textbook

An AI-native textbook for Physical AI and Humanoid Robotics, featuring an integrated RAG (Retrieval-Augmented Generation) chatbot that answers questions using only the textbook content.

## Features

- **6 Comprehensive Chapters** covering Physical AI, Humanoid Robotics, ROS 2, Digital Twins, VLA Systems, and a Capstone Project
- **RAG Chatbot** that answers questions with citations from the textbook
- **Select-to-Ask** feature for asking AI about selected text
- **Dark Mode** support with system preference detection
- **Mobile Responsive** design

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Frontend (Docusaurus)                │
│  - Static site with 6 chapters                          │
│  - ChatBot component (floating button + drawer)         │
│  - SelectToAsk component (text selection → Ask AI)      │
│  - Deployed to GitHub Pages                             │
└─────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────┐
│                   Backend (FastAPI)                     │
│  - /chat endpoint (RAG pipeline)                        │
│  - /chat/context endpoint (selected text context)       │
│  - /health endpoint (dependency checks)                 │
│  - Rate limiting (100 req/hour per IP)                  │
│  - Deployed to Railway/Vercel                           │
└─────────────────────────────────────────────────────────┘
        │                    │                    │
        ▼                    ▼                    ▼
┌──────────────┐   ┌──────────────┐   ┌──────────────┐
│ Qdrant Cloud │   │    Groq      │   │ Neon (Opt)   │
│ (Vectors)    │   │  (Llama 3)   │   │ (Metadata)   │
└──────────────┘   └──────────────┘   └──────────────┘
```

## Quick Start

### Prerequisites

- Node.js 20+ (for Docusaurus)
- Python 3.11+ (for backend)
- Git

### 1. Clone the Repository

```bash
git clone https://github.com/your-org/Hackathon-book.git
cd Hackathon-book
```

### 2. Set Up the Frontend

```bash
cd website
npm install
npm start
```

Open http://localhost:3000 to view the textbook.

### 3. Set Up the Backend

```bash
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Configure environment
cp .env.example .env
# Edit .env with your API keys

# Start the server
uvicorn src.main:app --reload --port 8000
```

### 4. Set Up External Services

1. **Qdrant Cloud** (https://cloud.qdrant.io)
   - Create a free cluster
   - Get your URL and API key
   - Run: `python scripts/setup_qdrant.py`

2. **Groq** (https://console.groq.com)
   - Get your API key

3. **Neon** (https://neon.tech) - Optional
   - Create a database
   - Run the schema: `scripts/setup_neon.sql`

### 5. Ingest Content

```bash
cd backend
python scripts/ingest.py --docs-path ../docs
```

## Project Structure

```
Hackathon-book/
├── docs/                  # Textbook chapters (Markdown)
│   ├── intro.md
│   ├── humanoid-basics.md
│   ├── ros2-fundamentals.md
│   ├── digital-twin.md
│   ├── vla-systems.md
│   └── capstone.md
├── website/               # Docusaurus frontend
│   ├── src/
│   │   ├── components/
│   │   │   ├── ChatBot/   # Chat UI components
│   │   │   └── SelectToAsk/ # Text selection feature
│   │   └── theme/         # Theme customizations
│   └── docusaurus.config.ts
├── backend/               # FastAPI backend
│   ├── src/
│   │   ├── api/           # API endpoints
│   │   ├── services/      # Business logic
│   │   └── models/        # Pydantic schemas
│   └── scripts/           # Setup and ingestion
└── specs/                 # Feature specifications
```

## Configuration

### Environment Variables

**Backend (.env)**
```env
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-key
GROQ_API_KEY=your-key
DATABASE_URL=postgresql://...  # Optional
CORS_ORIGINS=http://localhost:3000
```

**Frontend (.env)**
```env
REACT_APP_API_URL=http://localhost:8000
```

## Deployment

### Frontend (GitHub Pages)

```bash
cd website
npm run build
npm run deploy
```

### Backend (Railway)

1. Connect your GitHub repository
2. Set environment variables
3. Deploy automatically on push

## API Documentation

Once the backend is running, visit http://localhost:8000/docs for the OpenAPI documentation.

### Endpoints

- `GET /health` - Health check
- `POST /chat` - Send a chat message
- `POST /chat/context` - Send a message with selected text

## Development

### Frontend Development

```bash
cd website
npm start          # Start dev server
npm run build      # Production build
npm run typecheck  # TypeScript check
```

### Backend Development

```bash
cd backend
source venv/bin/activate
uvicorn src.main:app --reload
pytest tests/      # Run tests
```

## License

MIT License - See LICENSE file for details.
"# AI-book" 
