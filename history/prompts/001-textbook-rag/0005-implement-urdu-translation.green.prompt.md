---
id: 0005
title: Implement Urdu Translation for Textbook
stage: green
date: 2025-12-14
surface: agent
model: claude-opus-4-5-20251101
feature: 001-textbook-rag
branch: main
user: user
command: claude --continue
labels: ["i18n", "urdu", "translation", "us4"]
links:
  spec: specs/001-textbook-rag/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - website/i18n/ur/docusaurus-plugin-content-docs/current/intro.md
  - website/i18n/ur/docusaurus-plugin-content-docs/current/humanoid-basics.md
  - website/i18n/ur/docusaurus-plugin-content-docs/current/ros2-fundamentals.md
  - website/i18n/ur/docusaurus-plugin-content-docs/current/digital-twin.md
  - website/i18n/ur/docusaurus-plugin-content-docs/current/vla-systems.md
  - website/i18n/ur/docusaurus-plugin-content-docs/current/capstone.md
  - website/docusaurus.config.ts
  - specs/001-textbook-rag/tasks.md
tests:
  - null
---

## Prompt

Phase 6: User Story 4 - Urdu Translation (T051-T059)

## Response snapshot

Implemented complete Urdu translation for the Physical AI & Humanoid Robotics textbook:

1. **Directory Structure (T051)**: Created `website/i18n/ur/docusaurus-plugin-content-docs/current/` directory structure for Docusaurus i18n.

2. **Translated all 6 chapters (T052-T057)**:
   - `intro.md` - فزیکل AI کا تعارف
   - `humanoid-basics.md` - ہیومنائڈ روبوٹکس کی بنیادیات
   - `ros2-fundamentals.md` - ROS 2 بنیادیات
   - `digital-twin.md` - ڈیجیٹل ٹوئن سمیولیشن
   - `vla-systems.md` - Vision-Language-Action سسٹمز
   - `capstone.md` - کیپسٹون پروجیکٹ

3. **i18n Configuration (T058)**: Updated `docusaurus.config.ts` with:
   - Added 'ur' locale with RTL direction
   - Configured locale labels (English, اردو)
   - Set htmlLang to 'ur-PK'

4. **Language Switcher (T059)**: Added `localeDropdown` to navbar items for easy language switching.

All translations maintain the same structure, code examples, and technical content while adapting explanatory text to Urdu with appropriate RTL layout support.

## Outcome

- ✅ Impact: User Story 4 complete - Urdu translation fully implemented
- 🧪 Tests: Manual verification pending (build test)
- 📁 Files: 8 files created/modified
- 🔁 Next prompts: Build and verify Urdu locale works correctly
- 🧠 Reflection: Parallel file creation efficient; Docusaurus i18n setup straightforward

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Test RTL rendering in browser
