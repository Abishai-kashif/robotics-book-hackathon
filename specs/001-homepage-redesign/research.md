# Research: Homepage Redesign

**Feature**: 001-homepage-redesign
**Date**: 2025-12-27
**Status**: Complete

## Overview

This document consolidates research decisions for implementing the homepage redesign. Since the technical stack is already established (Docusaurus 3.9.2 + React 19), research focused on design patterns, content strategy, and Docusaurus best practices.

## Key Research Areas

### 1. Docusaurus Homepage Customization Patterns

**Decision**: Modify existing `src/pages/index.js` and `src/components/HomepageFeatures/index.js` in-place

**Rationale**:
- Docusaurus encourages overriding default components rather than creating parallel structures
- The `src/pages/index.js` file takes precedence over theme defaults
- Minimal changes to existing architecture reduce risk and complexity
- Component reusability maintained through existing `HomepageFeatures` component

**Alternatives Considered**:
- Swizzling Docusaurus theme components: Rejected because it creates maintenance burden with framework updates
- Creating custom theme: Rejected as overkill for a single page change
- Using MDX for homepage: Rejected because React components provide better control for interactive cards

**References**:
- Docusaurus documentation: https://docusaurus.io/docs/creating-pages
- React component patterns in Docusaurus: https://docusaurus.io/docs/markdown-features/react

---

### 2. Module Card Content & Navigation Strategy

**Decision**: Create 4 module cards representing the main learning paths, plus 1 chatbot feature card

**Module Structure**:
1. **Module 1**: "Robotic Nervous System (ROS 2)" → links to `/docs/ros2-fundamentals`
2. **Module 2**: "Digital Twin (Simulation)" → links to `/docs/gazebo-simulation`
3. **Module 3**: "AI-Robot Brain (NVIDIA Isaac)" → links to `/docs/nvidia-isaac-platform`
4. **Module 4**: "Vision-Language-Action (VLA)" → links to `/docs/introduction-to-vla-systems`
5. **Feature Card**: "AI Learning Assistant" → describes chatbot capability

**Rationale**:
- Aligns with existing sidebar structure in `sidebars.js`
- Each module has a clear entry point (first chapter)
- 4 modules + 1 feature = 5 cards fits well in grid layout (responsive: 1 col mobile, 2 cols tablet, 3 cols desktop with 2nd row)
- Links to actual documentation paths that already exist

**Alternatives Considered**:
- 3-card layout (combine modules): Rejected because each module is substantial and deserves prominence
- Links to module landing pages: Rejected because no dedicated landing pages exist; direct chapter links better
- Include hardware/capstone sections: Rejected because they're not complete/primary modules per sidebars.js

**Content Sources**:
- Module titles from `book-source/sidebars.js` lines 32-73
- Descriptions inferred from module focus and existing chapter titles

---

### 3. Hero Section Call-to-Action Design

**Decision**: Single primary CTA button "Start Learning" linking to `/docs/intro`

**Rationale**:
- `/docs/intro` is the natural starting point per sidebar configuration
- Single CTA reduces decision paralysis for new visitors
- "Start Learning" is action-oriented and education-focused
- Can add secondary button later if needed (e.g., "Browse Modules")

**Alternatives Considered**:
- Multiple CTAs ("Start", "Modules", "About"): Rejected to maintain focus and simplicity
- Link to first module directly: Rejected because intro page provides context and overview
- "Get Started" vs "Start Learning": Chose "Start Learning" for educational context clarity

---

### 4. Visual Design & Icons Strategy

**Decision**: Use emoji icons initially, migrate to custom SVGs/icons if needed in future

**Rationale**:
- Emoji provides immediate visual distinction without asset creation overhead
- Consistent cross-platform support in modern browsers
- Can be replaced with SVG icons without code structure changes
- Module-appropriate emojis available: 🤖 (ROS2), 🎮 (Simulation), 🧠 (Isaac), 👁️ (VLA), 💬 (Chatbot)

**Alternatives Considered**:
- Reuse existing Docusaurus SVGs: Rejected because they're generic (mountain, tree, React logo)
- Create custom SVG set immediately: Rejected as premature; validate layout/content first
- Use icon library (FontAwesome, Material Icons): Rejected to avoid new dependencies

**Migration Path**: If custom icons needed, replace emoji strings with `<svg>` or `<Icon>` components without changing card structure

---

### 5. Responsive Design Breakpoints

**Decision**: Follow Docusaurus/Infima default breakpoints and use existing CSS module system

**Breakpoints** (from Docusaurus Infima):
- Mobile: < 768px (1 column)
- Tablet: 768px - 996px (2 columns)
- Desktop: > 996px (3 columns with 5 cards wrapping to 2 rows)

**Rationale**:
- Consistency with existing Docusaurus theme
- No custom media queries needed; use `.row` and `.col--*` classes from Infima
- Existing `HomepageFeatures/styles.module.css` provides starting point

**Alternatives Considered**:
- Custom breakpoints: Rejected for consistency
- Fixed grid (no responsive): Rejected for mobile usability requirement (FR-007)
- Flexbox vs Grid: Using Infima's flexbox-based grid system (what Docusaurus uses)

---

### 6. Accessibility Considerations

**Decision**: Maintain semantic HTML, ARIA labels where needed, ensure keyboard navigation

**Implementation Checklist**:
- Use `<Link>` component from `@docusaurus/Link` (handles keyboard nav)
- Maintain heading hierarchy: `<h1>` for hero title, `<h2>` for section titles, `<h3>` for card titles
- Ensure sufficient color contrast (inherit from Docusaurus theme)
- Provide alt text for any images/icons added
- Test keyboard-only navigation (Tab, Enter)

**Rationale**:
- Success criteria SC-006 requires WCAG 2.1 Level AA compliance
- Docusaurus theme already provides accessible foundation
- Semantic HTML + proper heading levels = screen reader friendly

**Tools for Validation**:
- Chrome DevTools Lighthouse accessibility audit
- axe DevTools browser extension
- Keyboard navigation manual testing

---

### 7. Performance Optimization Strategy

**Decision**: Minimal JavaScript, static content, no external API calls, lazy load images if added

**Rationale**:
- Homepage is entry point; must be fast (SC-005: maintain 90% performance)
- Docusaurus SSG (Static Site Generation) already optimizes bundle
- Card content is static text; no dynamic data fetching needed
- No interactive features beyond navigation links

**Alternatives Considered**:
- Add animations/transitions: Deferred to future iteration if needed
- Dynamic content loading: Not needed; module structure is stable
- Image optimization: Will use if custom images added (Docusaurus has built-in optimization)

---

### 8. Chatbot Integration Approach

**Decision**: Add informational card about chatbot; assume chatbot widget already exists in layout

**Rationale**:
- FR-009 requires chatbot awareness, not implementation
- README.md indicates chatbot integration already exists (backend + frontend widget)
- Homepage card serves as promotion/education, not functional integration
- Card description: "Ask questions about any topic and get answers with citations from the textbook"

**Alternatives Considered**:
- Embed chatbot widget on homepage: Rejected; may distract from module navigation
- Add "Try Chatbot" button: Deferred; widget likely activates from sidebar/footer already
- Detailed chatbot documentation link: Deferred; simple awareness sufficient for P3 priority

---

## Technical Decisions Summary

| Decision Area | Choice | Rationale |
|---------------|--------|-----------|
| File Structure | Modify existing `index.js` + `HomepageFeatures/index.js` | Follows Docusaurus patterns, minimal changes |
| Module Cards | 4 module cards + 1 feature card (5 total) | Aligns with sidebar structure, clear learning paths |
| Hero CTA | "Start Learning" → `/docs/intro` | Single action, natural entry point |
| Icons | Emoji initially | Fast, no dependencies, easy to replace |
| Layout | Docusaurus Infima grid (1/2/3 cols) | Responsive, consistent with theme |
| Accessibility | Semantic HTML + WCAG 2.1 AA | Required by SC-006, use Docusaurus foundation |
| Performance | Static content, no JS overhead | Maintains current performance (SC-005) |
| Chatbot | Informational card only | Awareness (FR-009), not implementation |

---

## Implementation Readiness

✅ All research complete
✅ No external dependencies to add
✅ No NEEDS CLARIFICATION items remaining
✅ Technical approach validated against existing codebase
✅ Ready for Phase 1 (data-model.md and contracts)

## Next Steps

1. **Phase 1**: Define data model (module card structure) and component contracts
2. Generate `quickstart.md` with development setup
3. Proceed to `/sp.tasks` for implementation task breakdown
