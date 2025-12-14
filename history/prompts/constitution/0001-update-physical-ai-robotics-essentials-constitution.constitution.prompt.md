---
id: 0001
title: Update Physical AI Robotics Essentials Constitution
stage: constitution
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: unknown
command: /sp.constitution
labels: ["constitution", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md
tests:
- none
---

## Prompt

```text
#Project Name: Physical AI & Humanoid Robotics — Essentials

##Purpose:
Create a short, clean, professional AI-Native textbook based on the Physical AI & Humanoid Robotics course.
The book must serve as a fast, simple, high-quality learning resource built with a modern Docusaurus UI
and a fully integrated free-tier RAG chatbot.

##Scope:
- 6 short chapters:
  1. Introduction to Physical AI
  2. Basics of Humanoid Robotics
  3. ROS 2 Fundamentals
  4. Digital Twin Simulation (Gazebo + Isaac)
  5. Vision-Language-Action Systems
  6. Capstone: Simple AI-Robot Pipeline
- Clean UI
- Free-tier friendly
- Lightweight embeddings

##Core Principles:
- Simplicity
- Accuracy
- Minimalism
- Fast builds
- Free-tier architecture
- RAG answers ONLY from book text

##Key Features:
- Docusaurus textbook
- RAG chatbot (Qdrant + Neon + FastAPI)
- Select-text → Ask AI
- Optional Urdu / Personalize features

##Constraints:
- No heavy GPU usage
- Minimal embeddings

##Success Criteria:
- Build success
- Accurate chatbot
- Clean UI
- Smooth GitHub Pages deployment

Generate full constitution.
```

## Response snapshot

The project constitution was updated to version 1.0.0, incorporating the provided project brief, core principles, overview, and governance rules. Several dependent templates have been flagged for future alignment.

## Outcome

- ✅ Impact: The project constitution document has been created, establishing core principles and project definitions.
- 🧪 Tests: None
- 📁 Files: One file modified: .specify/memory/constitution.md
- 🔁 Next prompts: Review the updated constitution and make any necessary changes to the flagged templates: plan-template.md, spec-template.md, tasks-template.md, sp.plan.md, sp.specify.md, and sp.checklist.md.
- 🧠 Reflection: Successfully updated the project constitution and identified dependent templates requiring synchronization. The process highlighted the need for explicit placeholder mapping in the initial template.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Ensure all templates and command files are updated to reflect the new constitution and its specific principles.
