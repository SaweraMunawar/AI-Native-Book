# Feature Specification: AI-Native Textbook with RAG Chatbot

**Feature Branch**: `001-textbook-rag`
**Created**: 2025-12-09
**Status**: Draft
**Input**: User description: "AI-native textbook with RAG chatbot for Physical AI robotics curriculum with 6 chapters, Docusaurus frontend, Qdrant + Neon backend, free-tier embeddings, optional Urdu translation and personalization"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Browse Textbook Content (Priority: P1)

A student or learner visits the textbook website to study Physical AI and Humanoid Robotics concepts. They navigate through chapters sequentially or jump to specific topics using the auto-generated sidebar.

**Why this priority**: The core value of the product is educational content delivery. Without readable, navigable content, no other feature matters.

**Independent Test**: Can be fully tested by deploying a static Docusaurus site with all 6 chapters and verifying navigation works correctly. Delivers educational value immediately.

**Acceptance Scenarios**:

1. **Given** a deployed textbook site, **When** a user visits the homepage, **Then** they see a clean landing page with clear navigation to begin reading
2. **Given** a user is on any chapter page, **When** they view the sidebar, **Then** they see all 6 chapters with auto-generated section headings
3. **Given** a user is reading a chapter, **When** they reach the end, **Then** they see navigation to the next chapter

---

### User Story 2 - Ask Questions via RAG Chatbot (Priority: P2)

A learner has a question about content they're reading. They type their question into the chatbot interface and receive an answer derived exclusively from the textbook content.

**Why this priority**: The RAG chatbot differentiates this from a simple static textbook. It provides interactive learning but depends on content being available first (P1).

**Independent Test**: Can be tested by deploying the chatbot backend with embedded textbook content and verifying questions receive accurate, book-sourced answers.

**Acceptance Scenarios**:

1. **Given** a user is on any page, **When** they open the chatbot and type a question about covered topics, **Then** they receive an answer sourced from the textbook content within 5 seconds
2. **Given** a user asks a question outside the book's scope, **When** the system processes it, **Then** it responds indicating the topic is not covered in this textbook
3. **Given** a user receives an answer, **When** they view the response, **Then** they see a citation or reference to the relevant chapter/section

---

### User Story 3 - Select Text to Ask AI (Priority: P2)

A learner is reading a complex passage and wants clarification. They select the text and use a contextual "Ask AI" action to get an explanation based on the book content.

**Why this priority**: Enhances the reading experience by reducing friction to ask questions. Same priority as chatbot since it uses the same backend.

**Independent Test**: Can be tested by selecting text on any chapter page and verifying the AI responds with context-aware explanation.

**Acceptance Scenarios**:

1. **Given** a user is reading chapter content, **When** they select a passage of text, **Then** an "Ask AI" option appears
2. **Given** a user clicks "Ask AI" with selected text, **When** the system processes it, **Then** the chatbot opens pre-filled with the selected text as context
3. **Given** selected text contains technical jargon, **When** the AI responds, **Then** it provides explanation using only information from the textbook

---

### User Story 4 - Read Content in Urdu (Priority: P3 - Optional)

A learner who prefers Urdu can switch the interface language to read translated content.

**Why this priority**: Accessibility feature that expands reach but is not required for core functionality. Can be added after main features work.

**Independent Test**: Can be tested by adding translated markdown files and verifying language switcher displays them correctly.

**Acceptance Scenarios**:

1. **Given** a user is on any page, **When** they select Urdu from language options, **Then** the page content displays in Urdu
2. **Given** a user is reading in Urdu, **When** they ask the chatbot a question in Urdu, **Then** they receive a response in Urdu

---

### User Story 5 - Personalize Learning Experience (Priority: P3 - Optional)

A learner can customize their experience by setting preferences like dark mode, font size, or bookmarking chapters.

**Why this priority**: Nice-to-have feature that improves user experience but doesn't affect core learning delivery.

**Independent Test**: Can be tested by implementing preference toggles and verifying settings persist across sessions.

**Acceptance Scenarios**:

1. **Given** a user visits the site, **When** they toggle dark mode, **Then** the interface switches to dark theme and persists on refresh
2. **Given** a user is reading, **When** they bookmark a section, **Then** they can access their bookmarks from a dedicated page

---

### Edge Cases

- What happens when the chatbot backend is unavailable? Display a friendly message indicating the feature is temporarily unavailable while allowing continued reading.
- How does the system handle very long selected text for "Ask AI"? Truncate to a reasonable limit (e.g., 500 characters) with a notice.
- What happens when a chapter markdown file is malformed? The build process should fail fast with clear error messages.
- How does the system handle concurrent users on free tier? Design for graceful degradation with rate limiting and queue management.

## Requirements *(mandatory)*

### Functional Requirements

**Content Delivery**
- **FR-001**: System MUST render 6 chapters as navigable Docusaurus pages: Introduction to Physical AI, Basics of Humanoid Robotics, ROS 2 Fundamentals, Digital Twin Simulation (Gazebo + Isaac), Vision-Language-Action Systems, and Capstone
- **FR-002**: System MUST auto-generate sidebar navigation from markdown file structure
- **FR-003**: System MUST support markdown content with code blocks, images, and embedded diagrams
- **FR-004**: System MUST provide next/previous chapter navigation on each page

**RAG Chatbot**
- **FR-005**: System MUST provide a chatbot interface accessible from all pages via a floating button (bottom-right) that opens a slide-out drawer/modal
- **FR-006**: System MUST embed all textbook content into a vector database for retrieval
- **FR-007**: System MUST retrieve relevant passages before generating responses
- **FR-008**: System MUST generate answers using ONLY retrieved textbook content (no external knowledge)
- **FR-009**: System MUST include source references (chapter/section) in chatbot responses
- **FR-010**: System MUST handle questions outside book scope with appropriate "not covered" responses
- **FR-010a**: System MUST display a confidence disclaimer (e.g., "Based on limited context...") when retrieval confidence is below threshold, while still providing an answer

**Select-to-Ask Feature**
- **FR-011**: System MUST detect text selection on chapter pages
- **FR-012**: System MUST display "Ask AI" action when text is selected
- **FR-013**: System MUST pass selected text as context to the chatbot

**Infrastructure**
- **FR-014**: System MUST operate within free-tier limits of hosting providers
- **FR-015**: System MUST use lightweight embeddings suitable for free-tier compute
- **FR-016**: System MUST support static site deployment (GitHub Pages compatible)

**Optional - Urdu Translation**
- **FR-017**: System MAY support Urdu language content via i18n configuration
- **FR-018**: System MAY allow language switching between English and Urdu

**Optional - Personalization**
- **FR-019**: System MAY support dark/light theme toggle
- **FR-020**: System MAY persist user preferences in local storage

### Key Entities

- **Chapter**: A major section of the textbook with title, content (markdown), order position, and child sections
- **Section**: A subsection within a chapter with heading level, content, and parent chapter reference
- **Embedding**: Vector representation of textbook content chunks with source reference (chapter, section, paragraph)
- **ChatMessage**: User question and system response pair with timestamp, source citations, and session reference
- **UserPreference**: Optional stored settings including theme, language, and bookmarks

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All 6 chapters are accessible and readable within 3 clicks from the homepage
- **SC-002**: Sidebar accurately reflects all chapter and section headings with no manual configuration
- **SC-003**: Chatbot responds to questions within 5 seconds under normal load
- **SC-004**: 90% of chatbot responses to in-scope questions contain accurate information from the textbook (verified by human review of sample queries)
- **SC-005**: 100% of chatbot responses include citation to source chapter/section
- **SC-006**: System operates without paid tier upgrades for up to 100 daily active users
- **SC-007**: Site builds successfully and deploys to GitHub Pages without errors
- **SC-008**: Page load time under 3 seconds on standard broadband connection
- **SC-009**: "Ask AI" feature activates within 500ms of text selection
- **SC-010**: Users can complete reading any chapter without encountering broken links or missing content

## Clarifications

### Session 2025-12-09

- Q: When the RAG system retrieves passages but with low confidence scores (partial matches), how should the chatbot respond? → A: Answer with disclaimer when confidence is below threshold (e.g., "Based on limited context...")
- Q: Where should the chatbot interface be positioned on the page? → A: Floating button (bottom-right) that opens a slide-out drawer/modal
- Q: Which LLM provider should be used for generating chatbot responses from retrieved context? → A: Groq (Llama 3) - Free tier with fast inference, good quality

## Assumptions

- Qdrant Cloud free tier provides sufficient storage for textbook embeddings (~100KB-1MB estimated)
- Neon PostgreSQL free tier provides sufficient compute for metadata and session storage
- Free-tier embedding APIs (e.g., HuggingFace Inference API) provide adequate quality for educational Q&A
- Groq free tier (Llama 3) provides sufficient rate limits and quality for chatbot response generation
- Textbook content totals approximately 50-100 pages across 6 chapters
- Primary audience is English-speaking; Urdu is a secondary optional language
- Users have modern browsers (Chrome, Firefox, Safari, Edge - last 2 versions)

## Out of Scope

- User authentication or accounts
- Progress tracking or learning analytics
- Interactive exercises or quizzes
- Video or multimedia content beyond images
- Mobile native applications
- Offline reading capability
- Multi-user collaboration features
- Payment or premium content
