# Data Model: AI-Native Textbook with RAG Chatbot

**Feature**: 001-textbook-rag
**Date**: 2025-12-09

## Overview

This document defines the data entities, their attributes, relationships, and storage locations for the textbook RAG system.

## Entity Diagram

```
┌─────────────────┐     ┌─────────────────┐
│    Chapter      │────<│    Section      │
├─────────────────┤     ├─────────────────┤
│ slug (PK)       │     │ id (PK)         │
│ title           │     │ chapter_slug    │
│ position        │     │ heading         │
│ content_path    │     │ level           │
└─────────────────┘     │ content         │
         │              └─────────────────┘
         │                      │
         ▼                      ▼
┌─────────────────┐     ┌─────────────────┐
│   Embedding     │     │  ChatMessage    │
├─────────────────┤     ├─────────────────┤
│ id (PK)         │     │ id (PK)         │
│ vector[384]     │     │ session_id (FK) │
│ chapter_slug    │     │ role            │
│ section_id      │     │ content         │
│ chunk_text      │     │ sources[]       │
│ chunk_index     │     │ confidence      │
└─────────────────┘     │ created_at      │
                        └─────────────────┘
                                │
                                ▼
                        ┌─────────────────┐
                        │  ChatSession    │
                        ├─────────────────┤
                        │ id (PK)         │
                        │ created_at      │
                        │ message_count   │
                        └─────────────────┘
```

## Entities

### Chapter (Static - Markdown Files)

Represents a major section of the textbook. Stored as markdown files in `docs/` directory.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| slug | string | PK, unique | URL-safe identifier (e.g., "ros2-fundamentals") |
| title | string | required | Display title from frontmatter |
| position | integer | required | Order in sidebar (1-6) |
| content_path | string | required | File path relative to docs/ |

**Validation Rules**:
- slug must be lowercase alphanumeric with hyphens only
- position must be 1-6 (6 chapters total)
- content_path must end in `.md`

**Source**: Derived from markdown frontmatter at build time

---

### Section (Static - Derived from Markdown)

Represents a subsection within a chapter, extracted from markdown headings.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | string | PK | `{chapter_slug}#{heading_slug}` |
| chapter_slug | string | FK → Chapter | Parent chapter reference |
| heading | string | required | Section heading text |
| level | integer | 2-4 | Heading level (h2=2, h3=3, h4=4) |
| content | string | required | Section text content (until next heading) |

**Validation Rules**:
- level must be 2, 3, or 4 (h1 reserved for chapter title)
- content must not be empty

**Source**: Parsed from markdown during ingestion

---

### Embedding (Vector Store - Qdrant)

Vector representation of text chunks for semantic search.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | uuid | PK | Qdrant point ID |
| vector | float[384] | required | MiniLM embedding vector |
| chapter_slug | string | required | Source chapter |
| section_id | string | nullable | Source section (if applicable) |
| chunk_text | string | required | Original text chunk |
| chunk_index | integer | required | Position within chapter |
| start_char | integer | required | Character offset in source |
| end_char | integer | required | End character offset |

**Validation Rules**:
- vector must have exactly 384 dimensions
- chunk_text length: 100-2000 characters
- chunk_index >= 0

**Storage**: Qdrant Cloud collection `textbook_embeddings`

**Qdrant Payload Schema**:
```json
{
  "chapter_slug": "ros2-fundamentals",
  "section_id": "ros2-fundamentals#nodes-and-topics",
  "chunk_text": "ROS 2 nodes communicate via topics...",
  "chunk_index": 5,
  "start_char": 1234,
  "end_char": 1756
}
```

---

### ChatSession (Relational - Neon PostgreSQL)

Represents a user's chat session for rate limiting and optional history.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | uuid | PK | Session identifier |
| created_at | timestamp | default NOW() | Session start time |
| message_count | integer | default 0 | Number of messages |
| client_hash | string | nullable | Hashed IP for rate limiting |

**Validation Rules**:
- message_count >= 0
- client_hash is SHA-256 hash (64 chars) or null

**Storage**: Neon PostgreSQL table `chat_sessions`

---

### ChatMessage (API Runtime - Not Persisted)

Represents a single message in a chat exchange. Only exists in API request/response, not persisted.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | uuid | PK | Message identifier |
| session_id | uuid | FK → ChatSession | Parent session |
| role | enum | "user" \| "assistant" | Message author |
| content | string | required | Message text |
| sources | Source[] | nullable | Citation references (assistant only) |
| confidence | enum | "high" \| "medium" \| "low" | Retrieval confidence |
| created_at | timestamp | default NOW() | Message timestamp |

**Source Sub-entity**:
| Field | Type | Description |
|-------|------|-------------|
| chapter_slug | string | Source chapter |
| section_id | string | Source section |
| chunk_text | string | Relevant excerpt (first 200 chars) |
| score | float | Similarity score 0-1 |

**Validation Rules**:
- content length: 1-2000 characters for user, 1-4000 for assistant
- sources required when role is "assistant" and confidence != "low"

---

### RateLimit (Relational - Neon PostgreSQL)

Tracks API usage for rate limiting.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| client_hash | string | PK | SHA-256 of client IP |
| request_count | integer | default 0 | Requests in window |
| window_start | timestamp | default NOW() | Current window start |

**Validation Rules**:
- request_count >= 0
- window resets after 1 hour

**Storage**: Neon PostgreSQL table `rate_limits`

---

### UserPreference (Browser - LocalStorage)

Optional user preferences stored client-side only.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| theme | enum | "light" \| "dark" | Color theme |
| language | enum | "en" \| "ur" | Content language |
| bookmarks | string[] | max 50 | Bookmarked section IDs |

**Storage**: Browser localStorage key `textbook_preferences`

**Validation Rules**:
- bookmarks array max length 50
- Invalid values reset to defaults

---

## State Transitions

### ChatSession Lifecycle

```
┌─────────┐    create    ┌──────────┐    message    ┌──────────┐
│  None   │─────────────>│  Active  │──────────────>│  Active  │
└─────────┘              └──────────┘               └──────────┘
                              │                          │
                              │ 30min timeout            │ 30min timeout
                              ▼                          ▼
                         ┌──────────┐              ┌──────────┐
                         │ Expired  │              │ Expired  │
                         └──────────┘              └──────────┘
```

- Sessions created on first message
- Sessions expire after 30 minutes of inactivity
- Expired sessions can still be queried but not extended

---

## Indexes

### Qdrant
- Vector index: HNSW on `vector` field (automatic)
- Payload index: `chapter_slug` for filtered search

### Neon PostgreSQL
```sql
CREATE INDEX idx_sessions_created ON chat_sessions(created_at);
CREATE INDEX idx_rate_limits_window ON rate_limits(window_start);
```

---

## Data Volume Estimates

| Entity | Count | Storage |
|--------|-------|---------|
| Chapters | 6 | ~500KB markdown |
| Sections | ~60 | Derived at runtime |
| Embeddings | ~400 | ~600KB in Qdrant |
| ChatSessions | ~100/day | ~10KB/day in Neon |
| RateLimits | ~100 | ~5KB |

**Total**: Well within free tier limits (1GB Qdrant, 0.5GB Neon)
