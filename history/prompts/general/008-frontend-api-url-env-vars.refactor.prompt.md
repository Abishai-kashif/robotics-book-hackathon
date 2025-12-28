---
id: "008"
title: "Refactor frontend API URL to environment variables"
stage: "refactor"
date: "2025-12-28"
surface: "agent"
model: "sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "Human"
command: "Refactor the entire frontend to use environment variables for the backend API URL instead of hard-coding it."
labels: ["frontend","refactor","environment-variables"]
links:
  spec: "null"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
  - "book-source/.env"
  - "book-source/src/components/ChatbotWidget/index.js"
tests:
  - "None"
---

## Prompt

Refactor the entire frontend to use environment variables for the backend API URL instead of hard-coding it.

## Response snapshot

Completed refactoring of frontend API URL configuration:

1. Created `book-source/.env` file with `REACT_APP_API_BASE_URL=http://localhost:8000/api/v1`
2. Updated `book-source/src/components/ChatbotWidget/index.js` to use `process.env.REACT_APP_API_BASE_URL` with localhost fallback
3. Verified `frontend/src/services/api_client.js` already has proper environment variable support

The `frontend/src/services/api_client.js` file (line 5) already uses:
```javascript
constructor(baseURL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000/api/v1')
```

The refactored book-source component now uses:
```javascript
const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000/api/v1';
```

## Outcome

- ✅ Impact: Both frontend locations (book-source widget and standalone frontend) now use environment variables for API URL configuration
- 🧪 Tests: None
- 📁 Files: 2 files created/modified
- 🔁 Next prompts: None
- 🧠 Reflection: The `frontend/src/services/api_client.js` already had proper environment variable support. Only the Docusaurus ChatbotWidget component needed refactoring.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add build-time validation for required environment variables in Docusaurus config
