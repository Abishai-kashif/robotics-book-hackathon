# Quickstart: Homepage Redesign Development

**Feature**: 001-homepage-redesign
**Date**: 2025-12-27
**Estimated Setup Time**: 5 minutes

## Prerequisites

- Node.js ≥ 20.0 (check: `node --version`)
- npm or yarn package manager
- Git (for branch management)
- Code editor (VS Code recommended)

## Quick Setup

### 1. Verify Branch

```bash
git status
# Should show: On branch 001-homepage-redesign
```

If not on the correct branch:
```bash
git checkout 001-homepage-redesign
```

### 2. Install Dependencies

```bash
cd book-source
npm install
```

Expected output: Dependencies installed from `package.json` (Docusaurus 3.9.2, React 19)

### 3. Start Development Server

```bash
npm run start
```

This will:
- Start Docusaurus dev server
- Open browser to `http://localhost:3000`
- Enable hot reload for instant changes

### 4. Verify Current State

Navigate to `http://localhost:3000` and confirm you see:
- **Current (Before Changes)**:
  - Title: "Physical AI & Humanoid Robotics"
  - Tagline: "Bridging the gap between digital AI and physical robots"
  - Button: "Docusaurus Tutorial - 5min ⏱️" ← **This will be replaced**
  - Three feature cards: "Easy to Use", "Focus on What Matters", "Powered by React" ← **These will be replaced**

## File Locations

All changes will be made in these files:

```
book-source/
├── src/
│   ├── pages/
│   │   ├── index.js           ← UPDATE: Hero CTA button
│   │   └── index.module.css   ← UPDATE (if needed): Custom styles
│   └── components/
│       └── HomepageFeatures/
│           ├── index.js        ← UPDATE: Replace FeatureList
│           └── styles.module.css ← UPDATE (if needed): Card styles
```

**Reference Only** (no changes needed):
```
book-source/
├── docusaurus.config.js   ← Title/tagline already correct
└── sidebars.js            ← Module structure reference
```

## Development Workflow

### 1. Make Changes

Edit the files listed above according to implementation tasks (see `tasks.md` when generated).

### 2. See Changes Instantly

Docusaurus dev server auto-reloads on file save. Check browser for updates.

### 3. Check for Errors

Watch terminal for:
- Build errors (red text)
- React warnings (yellow text)
- Link validation errors

### 4. Test Responsiveness

Use browser DevTools:
- Open DevTools (F12)
- Toggle device toolbar (Ctrl+Shift+M / Cmd+Shift+M)
- Test: Mobile (375px), Tablet (768px), Desktop (1920px)

### 5. Validate Accessibility

Run Lighthouse audit:
- Open DevTools → Lighthouse tab
- Select "Accessibility" only
- Generate report
- Target: Score ≥ 90

## Common Commands

```bash
# Start dev server
npm run start

# Build for production (validates links)
npm run build

# Serve production build locally
npm run serve

# Clear cache (if weird behavior)
npm run clear
```

## Troubleshooting

### Port Already in Use

If port 3000 is occupied:
```bash
# Kill process on port 3000 (Linux/Mac)
lsof -ti:3000 | xargs kill -9

# Or use different port
npm run start -- --port 3001
```

### Hot Reload Not Working

```bash
# Clear Docusaurus cache
npm run clear
npm run start
```

### Build Errors After Changes

1. Check console for specific error message
2. Verify all imports are correct
3. Ensure no syntax errors in JSX
4. Confirm `linkTo` paths exist in docs

### TypeScript Errors (if using TS)

This project uses plain JavaScript. If you see TS errors:
- Ignore them (project is JS, not TS)
- Or install `@docusaurus/module-type-aliases` for type hints

## Testing Checklist

Before committing changes:

- [ ] Dev server starts without errors
- [ ] Homepage loads at `http://localhost:3000`
- [ ] All 5 module cards display correctly
- [ ] CTA button says "Start Learning" (not "Docusaurus Tutorial")
- [ ] All card links navigate to correct pages
- [ ] Responsive layout works (mobile/tablet/desktop)
- [ ] Keyboard navigation works (Tab through cards and CTA)
- [ ] No console errors or warnings
- [ ] `npm run build` succeeds (validates all links)
- [ ] Lighthouse accessibility score ≥ 90

## Key Files Reference

### Module Card Data (To Be Updated)

**File**: `book-source/src/components/HomepageFeatures/index.js`

Current structure to replace:
```javascript
const FeatureList = [
  {
    title: 'Easy to Use',  // ← Replace with module titles
    Svg: require('@site/static/img/...').default,  // ← Replace with icon
    description: <>...</>,  // ← Replace with module descriptions
  },
  // ... more items
];
```

New structure (from data-model.md):
```javascript
const FeatureList = [
  {
    title: 'Robotic Nervous System (ROS 2)',
    icon: '🤖',
    description: <>Build robot control systems using ROS 2...</>,
    linkTo: '/docs/ros2-fundamentals',
  },
  // ... 4 more cards
];
```

### Hero CTA Button (To Be Updated)

**File**: `book-source/src/pages/index.js`

Current code to replace:
```javascript
<Link
  className="button button--secondary button--lg"
  to="/docs/intro">
  Docusaurus Tutorial - 5min ⏱️  // ← Change this
</Link>
```

New code:
```javascript
<Link
  className="button button--secondary button--lg"
  to="/docs/intro">
  Start Learning  // ← New text
</Link>
```

## Next Steps

1. **Read** `specs/001-homepage-redesign/data-model.md` for card content
2. **Review** `specs/001-homepage-redesign/contracts/component-interfaces.md` for component contracts
3. **Wait for** `tasks.md` generation (via `/sp.tasks` command)
4. **Implement** tasks in order (P1 → P2 → P3)
5. **Test** at each step using checklist above
6. **Commit** when all tests pass

## Resources

- [Docusaurus Pages Documentation](https://docusaurus.io/docs/creating-pages)
- [React Hooks Reference](https://react.dev/reference/react)
- [WCAG 2.1 Guidelines](https://www.w3.org/WAI/WCAG21/quickref/)

## Support

If you encounter issues:
1. Check terminal output for error messages
2. Review `specs/001-homepage-redesign/research.md` for design decisions
3. Refer to `specs/001-homepage-redesign/contracts/component-interfaces.md` for component behavior
4. Consult Docusaurus documentation for framework-specific issues

---

**Ready to implement?** Run `/sp.tasks` to generate the implementation task breakdown.
