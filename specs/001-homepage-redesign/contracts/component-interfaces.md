# Component Interfaces: Homepage Redesign

**Feature**: 001-homepage-redesign
**Date**: 2025-12-27
**Type**: React Component Contracts

## Overview

This document defines the interface contracts for React components modified/created during the homepage redesign. Since this is a frontend-only feature with no API endpoints, contracts focus on component props, behavior, and rendering guarantees.

---

## Component: `HomepageFeatures`

**File**: `book-source/src/components/HomepageFeatures/index.js`

### Purpose
Renders a grid of module/feature cards that link to documentation sections.

### Interface

#### Props
```typescript
interface HomepageFeaturesProps {
  // No external props - component is self-contained
}
```

#### Internal Data Structure
```typescript
interface ModuleCard {
  title: string;           // Display name (required, non-empty)
  icon: string;            // Emoji or future: React component (required)
  description: ReactNode;  // JSX content (required)
  linkTo: string;          // Docusaurus route (required, must start with '/')
}

const FeatureList: ModuleCard[] = [...]; // Array of 5 cards
```

### Behavior Contract

**MUST**:
1. Render exactly 5 module cards in the order defined in `FeatureList`
2. Wrap each card in a Docusaurus `<Link>` component for navigation
3. Use responsive grid layout:
   - Mobile (<768px): 1 column
   - Tablet (768-996px): 2 columns
   - Desktop (>996px): 3 columns (wraps to 2 rows)
4. Apply `styles.features` class for consistent spacing
5. Each card must be keyboard-navigable (inherited from `<Link>`)
6. Each card must have `role="img"` on icon element for accessibility

**MUST NOT**:
1. Fetch external data (all content static)
2. Maintain internal state (pure presentation component)
3. Perform side effects on render
4. Break on missing/invalid data (build-time validation)

### Rendering Guarantees

**Given** a valid `FeatureList` array:
- **Then** renders 5 clickable cards
- **And** each card navigates to specified `linkTo` route on click
- **And** layout is responsive per breakpoint specifications
- **And** maintains accessibility (keyboard nav, semantic HTML)

**Given** an invalid `linkTo` path:
- **Then** Docusaurus build fails (link validation)

### Dependencies
- `@docusaurus/Link`: Navigation component
- `@theme/Heading`: Heading component with theme support
- `clsx`: CSS class composition
- `./styles.module.css`: Component styles

---

## Component: `Feature`

**File**: `book-source/src/components/HomepageFeatures/index.js` (internal)

### Purpose
Renders a single module/feature card (internal component used by `HomepageFeatures`).

### Interface

#### Props
```typescript
interface FeatureProps {
  icon: string;            // Emoji character
  title: string;           // Card title
  description: ReactNode;  // Card description (JSX)
  linkTo: string;          // Navigation destination
}
```

### Behavior Contract

**MUST**:
1. Render as a single column in grid (`.col.col--4` for 3-column desktop layout)
2. Display icon centered with `text--center` alignment
3. Display title as `<Heading as="h3">` for accessibility
4. Display description in centered, padded container
5. Be wrapped in `<Link>` by parent component

**MUST NOT**:
1. Handle navigation directly (parent handles wrapping with `<Link>`)
2. Apply external margins (parent grid handles spacing)

### Rendering Guarantees

**Given** valid props:
- **Then** renders card with icon, title, and description
- **And** maintains consistent visual styling per `styles.module.css`
- **And** scales text appropriately on small screens

---

## Component: `HomepageHeader`

**File**: `book-source/src/pages/index.js`

### Purpose
Renders the hero section with title, tagline, and CTA button.

### Interface

#### Props
```typescript
interface HomepageHeaderProps {
  // No external props - uses Docusaurus context
}
```

#### Data Source
```typescript
const { siteConfig } = useDocusaurusContext();
// Accesses:
// - siteConfig.title: "Physical AI & Humanoid Robotics"
// - siteConfig.tagline: "Bridging the gap between digital AI and physical robots"
```

### Behavior Contract

**MUST**:
1. Render as `<header>` with `hero hero--primary` classes
2. Display `siteConfig.title` as `<Heading as="h1">`
3. Display `siteConfig.tagline` as `<p class="hero__subtitle">`
4. Include CTA button with text "Start Learning" linking to `/docs/intro`
5. Apply `styles.heroBanner` for custom styling
6. Use Docusaurus `<Link>` component for CTA button

**MUST NOT**:
1. Hardcode title/tagline (must use `siteConfig`)
2. Render without CTA button
3. Break responsive layout on mobile

### Rendering Guarantees

**Given** valid site configuration:
- **Then** renders hero with title, tagline, and CTA
- **And** CTA navigates to introduction page
- **And** maintains responsive design (stacks vertically on mobile)

### Dependencies
- `useDocusaurusContext`: Access site config
- `@docusaurus/Link`: CTA button navigation
- `@theme/Heading`: Theme-aware heading component
- `clsx`: Class composition

---

## Component: `Home` (Page Component)

**File**: `book-source/src/pages/index.js`

### Purpose
Main homepage component that composes header and features sections.

### Interface

#### Props
```typescript
interface HomeProps {
  // No props - top-level page component
}
```

### Behavior Contract

**MUST**:
1. Wrap all content in `<Layout>` component with:
   - `title`: Template string from `siteConfig.title`
   - `description`: Meta description for SEO
2. Render `<HomepageHeader />` as first child
3. Render `<HomepageFeatures />` inside `<main>` element
4. Use semantic HTML structure (header → main)

**MUST NOT**:
1. Include Docusaurus tutorial references
2. Render without `<Layout>` wrapper
3. Break SEO metadata

### Rendering Guarantees

**Given** valid component hierarchy:
- **Then** renders complete homepage with header and features
- **And** maintains Docusaurus layout (nav, footer)
- **And** includes proper SEO metadata
- **And** passes accessibility audit (semantic structure)

### Dependencies
- `useDocusaurusContext`: Site configuration
- `@theme/Layout`: Docusaurus layout wrapper
- `HomepageHeader`: Hero section component
- `HomepageFeatures`: Module cards component

---

## CSS Module: `index.module.css`

**File**: `book-source/src/pages/index.module.css`

### Purpose
Styles for homepage components (hero banner, buttons).

### Contract

**MUST**:
1. Define `.heroBanner` class with padding and background
2. Define `.buttons` class for CTA button container
3. Support responsive breakpoints:
   - Mobile: Reduced padding, single column
   - Desktop: Full padding, centered content
4. Maintain Docusaurus theme variables (colors, spacing)

---

## CSS Module: `HomepageFeatures/styles.module.css`

**File**: `book-source/src/components/HomepageFeatures/styles.module.css`

### Purpose
Styles for feature card grid.

### Contract

**MUST**:
1. Define `.features` class for grid container
2. Define `.featureSvg` class (currently for SVG icons, will support emoji)
3. Support responsive layout (1/2/3 columns)
4. Maintain consistent card spacing and alignment

---

## Navigation Contract

### Internal Links (All Components)

**MUST**:
1. Use `@docusaurus/Link` component for all internal navigation
2. Validate link paths at build time (Docusaurus feature)
3. Support keyboard navigation (Tab, Enter)
4. Provide hover/focus states (inherited from Docusaurus theme)

**Link Destinations**:
- Hero CTA: `/docs/intro`
- Module 1 Card: `/docs/ros2-fundamentals`
- Module 2 Card: `/docs/gazebo-simulation`
- Module 3 Card: `/docs/nvidia-isaac-platform`
- Module 4 Card: `/docs/introduction-to-vla-systems`
- Chatbot Card: `/docs/intro` (fallback; chatbot is global widget)

---

## Accessibility Contract

### All Components

**MUST**:
1. Use semantic HTML elements (`<header>`, `<main>`, `<nav>`)
2. Maintain heading hierarchy (h1 → h2 → h3)
3. Provide keyboard navigation support
4. Ensure color contrast meets WCAG 2.1 AA (inherited from theme)
5. Support screen readers (ARIA labels where needed)

**Validation**:
- Lighthouse accessibility score ≥ 90
- No ARIA violations in axe DevTools
- Keyboard-only navigation functional (Tab, Enter, Shift+Tab)

---

## Performance Contract

### All Components

**MUST**:
1. Render without external API calls (static content)
2. Bundle size: No significant increase (< 5KB added)
3. Lighthouse performance score: Maintain ≥ 90
4. First Contentful Paint: < 2 seconds on 3G

**Optimization**:
- No heavy JavaScript dependencies added
- Use Docusaurus code splitting (automatic)
- Lazy load images if added (future)

---

## Testing Contract

### Component Behavior

**Manual Testing Required**:
1. Visual regression: Homepage matches design intent
2. Responsive testing: Mobile (320px), Tablet (768px), Desktop (1920px)
3. Browser testing: Chrome, Firefox, Safari (latest versions)
4. Keyboard navigation: All cards and CTA accessible via keyboard
5. Link validation: All module links navigate to correct pages

**Build-Time Validation**:
1. Docusaurus build succeeds without errors
2. No broken links detected (Docusaurus validation)
3. No React warnings in console

**Accessibility Testing**:
1. Lighthouse audit: Accessibility ≥ 90
2. axe DevTools: No violations
3. Keyboard navigation: All interactive elements reachable

---

## Error Handling

### Build-Time Errors

**Condition**: Invalid `linkTo` path in `FeatureList`
**Behavior**: Docusaurus build fails with link validation error
**Resolution**: Correct the path to match existing documentation

**Condition**: Missing required prop in component
**Behavior**: React development warnings, potential runtime error
**Resolution**: Ensure all props in `FeatureList` are complete

### Runtime Errors

**Condition**: Missing `siteConfig` values
**Behavior**: Undefined title/tagline in hero section
**Resolution**: Verify `docusaurus.config.js` has `title` and `tagline`

---

## Versioning & Compatibility

**Docusaurus Version**: 3.9.2
**React Version**: 19.0.0
**Node Version**: ≥ 20.0

**Breaking Changes**:
- None - modifications extend existing components, no API changes

**Backward Compatibility**:
- ✅ Maintains Docusaurus layout and navigation
- ✅ Preserves existing routes and documentation links
- ✅ Compatible with existing chatbot integration
- ✅ No changes to build process or deployment

---

## Summary

This homepage redesign maintains strict contracts for:
- Component props and behavior
- Navigation and linking
- Accessibility and performance
- Responsive design and layout
- Build-time validation

All contracts align with Docusaurus best practices and ensure a maintainable, accessible, and performant homepage.
