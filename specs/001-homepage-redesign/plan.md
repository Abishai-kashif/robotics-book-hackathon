# Implementation Plan: Homepage Redesign for Physical AI & Humanoid Robotics Textbook

**Branch**: `001-homepage-redesign` | **Date**: 2025-12-27 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-homepage-redesign/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Replace the default Docusaurus template homepage with textbook-specific content that immediately communicates the purpose (Physical AI & Humanoid Robotics education) and provides clear navigation to the four main modules (ROS 2, Simulation, NVIDIA Isaac, VLA). The homepage will feature a hero section with the existing title/tagline, module preview cards with navigation links, chatbot feature awareness, and maintain responsive design across all devices.

## Technical Context

**Language/Version**: JavaScript/React 19.0.0 (Docusaurus 3.9.2 framework)
**Primary Dependencies**: @docusaurus/core 3.9.2, @docusaurus/preset-classic 3.9.2, React 19, clsx, prism-react-renderer
**Storage**: N/A (static content generation)
**Testing**: Manual testing across browsers and devices (Chrome, Firefox, Safari), accessibility testing with browser dev tools
**Target Platform**: Static site deployed to web (supports modern browsers per browserslist config)
**Project Type**: Web (Docusaurus static site generator with React components)
**Performance Goals**: Maintain current lighthouse scores (target >90), page load <2 seconds on 3G
**Constraints**: Must use existing Docusaurus theme system, maintain compatibility with existing chatbot integration, no breaking changes to navigation/footer
**Scale/Scope**: Single homepage file + one feature component, ~200 lines of JSX, 4 module cards

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Analysis

This feature is a **documentation/educational content update** that does NOT involve:
- Physical AI integration (constitution principle I)
- ROS 2 implementation (principle II)
- Simulation environments (principle III)
- Multi-platform robotics deployment (principle IV)
- Vision-Language-Action systems (principle V)
- Humanoid robot control (principle VI)
- Conversational AI in robots (principle VII - note: promotes existing chatbot UI only)
- Hardware-aware computational content (principle VIII)

**Constitution Compliance**: ✅ **EXEMPT**

**Rationale**: This feature modifies the textbook's **landing page UI** to improve educational content discovery. It does not change any robotics concepts, code examples, or technical content covered by the constitution. The constitution governs "content" (educational material about Physical AI and robotics) while this feature updates the "container" (homepage presentation layer).

**Gate Status**: **PASS** - Homepage redesign is a meta-feature (presentation layer) that supports but does not implement constitution-governed robotics content.

**Re-evaluation after Phase 1**: Constitution check remains valid; homepage design does not introduce technical content requiring constitution compliance.

## Project Structure

### Documentation (this feature)

```text
specs/001-homepage-redesign/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
book-source/
├── src/
│   ├── components/
│   │   └── HomepageFeatures/     # UPDATE: Replace feature list
│   │       ├── index.js
│   │       └── styles.module.css
│   └── pages/
│       ├── index.js               # UPDATE: Modify homepage component
│       └── index.module.css       # UPDATE: Adjust styles if needed
├── static/
│   └── img/                       # MAY ADD: Module icons/images
├── docusaurus.config.js           # REFERENCE: Already has correct title/tagline
└── sidebars.js                    # REFERENCE: Module structure for links
```

**Structure Decision**: This is a **web application** (Docusaurus site). We will modify existing files in `book-source/src/pages/` and `book-source/src/components/HomepageFeatures/` rather than creating new directories. The Docusaurus framework already provides the structure, we're updating content within it.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

N/A - No constitution violations. Feature is exempt as presentation layer.
