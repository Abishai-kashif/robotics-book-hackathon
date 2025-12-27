# Data Model: Homepage Redesign

**Feature**: 001-homepage-redesign
**Date**: 2025-12-27
**Status**: Complete

## Overview

This document defines the data structures used in the homepage redesign. Since this is a static content feature with no backend or database, the "data model" describes the component props and content structures used in React components.

## Entities

### 1. ModuleCard

Represents a learning module or feature displayed on the homepage.

**Attributes**:
- `title` (string, required): Display name of the module or feature
  - Validation: Non-empty, max 80 characters
  - Example: "Robotic Nervous System (ROS 2)"

- `icon` (string, required): Visual identifier for the card
  - Format: Emoji character or future: React component
  - Example: "🤖"

- `description` (ReactNode, required): Brief explanation of module content
  - Format: JSX/React element (allows text formatting)
  - Validation: Non-empty, recommended 50-150 characters
  - Example: `<>Learn to build robot control systems using ROS 2, the industry standard robot middleware...</>`

- `linkTo` (string, required): Navigation destination when card is clicked
  - Format: Docusaurus route path (must start with `/docs/` for documentation)
  - Validation: Must be valid internal route
  - Example: "/docs/ros2-fundamentals"

**Relationships**:
- N/A (independent cards, no parent-child relationships)

**State Transitions**:
- N/A (static content, no state changes)

**Validation Rules**:
1. All fields required (cannot be null/undefined)
2. `title` must be unique within the card array
3. `linkTo` must be a valid Docusaurus route (validated at build time)
4. `description` should be concise for card layout (overflow handling via CSS)

**Usage Context**:
- Array of 5 ModuleCard objects passed to `HomepageFeatures` component
- Rendered as clickable card grid on homepage

---

### 2. HeroContent

Represents the main hero section content on the homepage.

**Attributes**:
- `title` (string, required): Main headline
  - Source: `siteConfig.title` from `docusaurus.config.js`
  - Example: "Physical AI & Humanoid Robotics"

- `tagline` (string, required): Subtitle/description
  - Source: `siteConfig.tagline` from `docusaurus.config.js`
  - Example: "Bridging the gap between digital AI and physical robots"

- `ctaText` (string, required): Call-to-action button label
  - Example: "Start Learning"

- `ctaLink` (string, required): CTA button destination
  - Format: Docusaurus route path
  - Example: "/docs/intro"

**Relationships**:
- Consumed by `HomepageHeader` component
- Values sourced from `useDocusaurusContext()` hook

**State Transitions**:
- N/A (static configuration from docusaurus.config.js)

**Validation Rules**:
1. All values pulled from site config (cannot be overridden arbitrarily)
2. `ctaLink` must be valid route

---

## Component Data Contracts

### HomepageFeatures Component

**Input Props**:
```typescript
interface HomepageFeaturesProps {
  // No props - data defined internally in component
}
```

**Internal Data Structure**:
```typescript
interface ModuleCard {
  title: string;
  icon: string;
  description: React.ReactNode;
  linkTo: string;
}

const FeatureList: ModuleCard[] = [
  // Array of 5 cards
];
```

**Output Rendering**:
- Grid of clickable cards with responsive layout
- Each card wrapped in Docusaurus `<Link>` component for navigation

---

### HomepageHeader Component

**Input Props**:
```typescript
interface HomepageHeaderProps {
  // No explicit props - uses context
}
```

**Data Source**:
```typescript
const { siteConfig } = useDocusaurusContext();
// Accesses siteConfig.title and siteConfig.tagline
```

**Output Rendering**:
- Hero banner with title, tagline, and CTA button

---

### Home (Main Page Component)

**Input Props**:
```typescript
interface HomeProps {
  // No props - top-level page component
}
```

**Data Flow**:
1. Renders `<HomepageHeader />` (pulls from site config)
2. Renders `<HomepageFeatures />` (uses internal FeatureList data)
3. Wrapped in `<Layout>` with metadata

---

## Data Sources

### Static Configuration

**File**: `book-source/docusaurus.config.js`
- `title`: "Physical AI & Humanoid Robotics"
- `tagline`: "Bridging the gap between digital AI and physical robots"

**File**: `book-source/sidebars.js`
- Module structure and navigation paths
- Used to determine `linkTo` values for module cards

### Component-Defined Constants

**File**: `book-source/src/components/HomepageFeatures/index.js`
- `FeatureList` array: Hardcoded module card data
- Updated during implementation to reflect textbook modules

---

## No Backend/Database

This feature has:
- ❌ No API endpoints
- ❌ No database tables
- ❌ No external data fetching
- ❌ No user-generated content
- ✅ Static content only (defined at build time)

All data is embedded in React components and compiled into the static site bundle.

---

## Module Card Content Specification

### Card 1: ROS 2 Module
```javascript
{
  title: "Robotic Nervous System (ROS 2)",
  icon: "🤖",
  description: <>Build robot control systems using ROS 2, the industry-standard middleware for robotics applications.</>,
  linkTo: "/docs/ros2-fundamentals"
}
```

### Card 2: Simulation Module
```javascript
{
  title: "Digital Twin (Simulation)",
  icon: "🎮",
  description: <>Master Gazebo and Unity simulations to test robots virtually before real-world deployment.</>,
  linkTo: "/docs/gazebo-simulation"
}
```

### Card 3: NVIDIA Isaac Module
```javascript
{
  title: "AI-Robot Brain (NVIDIA Isaac)",
  icon: "🧠",
  description: <>Integrate AI workflows and perception systems using NVIDIA Isaac for intelligent robot behavior.</>,
  linkTo: "/docs/nvidia-isaac-platform"
}
```

### Card 4: VLA Module
```javascript
{
  title: "Vision-Language-Action (VLA)",
  icon: "👁️",
  description: <>Enable robots to understand natural language commands and translate them into physical actions.</>,
  linkTo: "/docs/introduction-to-vla-systems"
}
```

### Card 5: Chatbot Feature
```javascript
{
  title: "AI Learning Assistant",
  icon: "💬",
  description: <>Ask questions about any topic and get instant answers with citations from the textbook.</>,
  linkTo: "/docs/intro" // Generic link since chatbot is global widget
}
```

---

## Validation & Testing

**Build-Time Validation**:
- Docusaurus will fail build if `linkTo` paths don't exist
- React will warn if required props missing

**Runtime Validation**:
- None needed (static content)

**Content Validation Checklist**:
- [ ] All 5 cards have unique titles
- [ ] All icons render correctly (emoji support)
- [ ] All descriptions fit within card layout (manual visual test)
- [ ] All `linkTo` paths navigate to valid documentation pages
- [ ] Card order matches logical learning progression

---

## Migration Notes

**Current State** (default Docusaurus):
- 3 generic feature cards ("Easy to Use", "Focus on What Matters", "Powered by React")
- SVG icons from `@site/static/img/undraw_docusaurus_*.svg`

**Target State**:
- 5 textbook-specific cards (4 modules + 1 feature)
- Emoji icons (replaceable with SVGs later)

**Data Migration**:
- None required (no persisted data)
- Simple array replacement in `HomepageFeatures/index.js`
