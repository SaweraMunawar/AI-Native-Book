<!--
Sync Impact Report:
Version change: 0.0.0 → 1.0.0
Modified principles: Simplicity, Accuracy, Minimalism, Fast Builds, Free-tier Architecture, RAG Answers ONLY from Book Text
Added sections: Project Overview (Purpose, Scope, Key Features, Constraints, Success Criteria)
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md ⚠ pending
- .specify/templates/spec-template.md ⚠ pending
- .specify/templates/tasks-template.md ⚠ pending
- .claude/commands/sp.plan.md ⚠ pending
- .claude/commands/sp.specify.md ⚠ pending
- .claude/commands/sp.checklist.md ⚠ pending
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics — Essentials Constitution

## Core Principles

### I. Simplicity
MUST ensure all solutions are straightforward, easy to understand, and avoid unnecessary complexity. The textbook content, UI, and chatbot architecture will prioritize clarity and directness.

### II. Accuracy
MUST provide factually correct and up-to-date information across all chapters. The RAG chatbot MUST only return answers directly derivable from the book's text, maintaining high fidelity.

### III. Minimalism
MUST adhere to a lean design approach, focusing on essential features and content. This applies to codebase, UI design, and resource usage to support fast builds and free-tier deployment.

### IV. Fast Builds
MUST ensure the Docusaurus site builds quickly to support rapid iteration and deployment. Optimize build processes and minimize dependencies.

### V. Free-tier Architecture
MUST design the entire system (Docusaurus, RAG chatbot components) to be deployable and operational within free-tier limits of common cloud providers.

### VI. RAG Answers ONLY from Book Text
MUST ensure the RAG chatbot's responses are strictly confined to information present within the textbook content. No external knowledge or generative answers are permitted.

## Project Overview

### Purpose
To create a short, clean, professional AI-Native textbook based on the Physical AI & Humanoid Robotics course. The book must serve as a fast, simple, high-quality learning resource built with a modern Docusaurus UI and a fully integrated free-tier RAG chatbot.

### Scope
- 6 short chapters: Introduction to Physical AI, Basics of Humanoid Robotics, ROS 2 Fundamentals, Digital Twin Simulation (Gazebo + Isaac), Vision-Language-Action Systems, Capstone: Simple AI-Robot Pipeline.
- Clean UI
- Free-tier friendly
- Lightweight embeddings

### Key Features
- Docusaurus textbook
- RAG chatbot (Qdrant + Neon + FastAPI)
- Select-text → Ask AI
- Optional Urdu / Personalize features

### Constraints
- No heavy GPU usage
- Minimal embeddings

### Success Criteria
- Build success
- Accurate chatbot
- Clean UI
- Smooth GitHub Pages deployment

## Governance
This Constitution supersedes all other project practices. Amendments require a documented change, approval by core maintainers, and a clear migration plan for any affected systems. All Pull Requests and code reviews MUST verify compliance with these principles. Complexity MUST always be justified.

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
