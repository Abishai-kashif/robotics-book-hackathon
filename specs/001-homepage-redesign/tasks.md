---
description: "Implementation tasks for homepage redesign"
---

# Tasks: Homepage Redesign for Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/001-homepage-redesign/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/component-interfaces.md, quickstart.md

**Tests**: This feature uses **manual testing** (visual validation, browser testing, accessibility audits). No automated test tasks included.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Project Type**: Docusaurus static site (web app)
- **Source Root**: `book-source/src/`
- **Files Modified**: `pages/index.js`, `components/HomepageFeatures/index.js`, CSS modules
- All paths relative to repository root: `C:\robotics-book-2\`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and verification

- [x] T001 Verify current branch is `001-homepage-redesign` using `git status`
- [x] T002 Install Node.js dependencies in `book-source/` using `npm install`
- [ ] T003 Start Docusaurus dev server using `npm run start` in `book-source/`
- [ ] T004 Verify homepage loads at `http://localhost:3000` and shows default Docusaurus content

**Checkpoint**: Development environment ready - can see current homepage with "Docusaurus Tutorial" button and 3 generic cards

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Read existing code to understand structure before making changes

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T005 Read `book-source/src/pages/index.js` to understand current homepage structure
- [x] T006 Read `book-source/src/components/HomepageFeatures/index.js` to understand FeatureList structure
- [x] T007 Read `book-source/docusaurus.config.js` to verify title and tagline are correct
- [x] T008 Read `book-source/sidebars.js` to identify module structure and navigation paths
- [x] T009 Review `specs/001-homepage-redesign/data-model.md` for exact card content specifications

**Checkpoint**: Foundation ready - understand existing code structure and have exact content to implement

---

## Phase 3: User Story 1 - First-time Visitor Landing Experience (Priority: P1) 🎯 MVP

**Goal**: Replace default Docusaurus content with textbook-specific hero section and module cards so visitors immediately understand this is a Physical AI & Humanoid Robotics textbook

**Independent Test**: Navigate to homepage and verify: (1) title/tagline display correctly, (2) "Start Learning" button appears (not "Docusaurus Tutorial"), (3) 5 textbook module cards display (not 3 generic cards), (4) no "Docusaurus" product references visible

### Implementation for User Story 1

- [x] T010 [US1] Update hero CTA button in `book-source/src/pages/index.js` line 20-24: change text from "Docusaurus Tutorial - 5min ⏱️" to "Start Learning"
- [x] T011 [US1] Replace FeatureList array in `book-source/src/components/HomepageFeatures/index.js` lines 5-36 with 5 new module cards per data-model.md
- [x] T012 [US1] Update card 1 content in FeatureList: title="Robotic Nervous System (ROS 2)", icon="🤖", description per data-model.md, linkTo="/docs/ros2-fundamentals"
- [x] T013 [US1] Update card 2 content in FeatureList: title="Digital Twin (Simulation)", icon="🎮", description per data-model.md, linkTo="/docs/gazebo-simulation"
- [x] T014 [US1] Update card 3 content in FeatureList: title="AI-Robot Brain (NVIDIA Isaac)", icon="🧠", description per data-model.md, linkTo="/docs/nvidia-isaac-platform"
- [x] T015 [US1] Update card 4 content in FeatureList: title="Vision-Language-Action (VLA)", icon="👁️", description per data-model.md, linkTo="/docs/introduction-to-vla-systems"
- [x] T016 [US1] Update card 5 content in FeatureList: title="AI Learning Assistant", icon="💬", description per data-model.md, linkTo="/docs/intro"
- [x] T017 [US1] Update Feature component in `book-source/src/components/HomepageFeatures/index.js` lines 38-50 to accept `icon`, `title`, `description`, `linkTo` props instead of `Svg`
- [x] T018 [US1] Wrap each Feature component render with `<Link to={linkTo}>` in `book-source/src/components/HomepageFeatures/index.js` lines 52-63
- [x] T019 [US1] Update Feature component icon rendering: replace `<Svg className={styles.featureSvg} role="img" />` with `<span className={styles.featureIcon} role="img">{icon}</span>` to display emoji
- [ ] T020 [US1] Verify hot reload shows updated homepage with all 5 cards and new CTA button
- [ ] T021 [US1] Test all 5 card links navigate to correct documentation pages by clicking each one
- [ ] T022 [US1] Test "Start Learning" button navigates to `/docs/intro`
- [ ] T023 [US1] Search homepage HTML for "Docusaurus" text and verify only config metadata remains (no visible references)

**Manual Testing Checklist for US1**:
- [ ] Homepage displays "Physical AI & Humanoid Robotics" title
- [ ] Tagline shows "Bridging the gap between digital AI and physical robots"
- [ ] "Start Learning" button visible (not "Docusaurus Tutorial")
- [ ] Exactly 5 cards visible with correct titles and icons
- [ ] All cards clickable and navigate to correct pages
- [ ] No visible "Docusaurus" product references on page
- [ ] Page layout looks reasonable (not broken)

**Checkpoint**: At this point, User Story 1 should be fully functional - homepage shows textbook content with working navigation

---

## Phase 4: User Story 2 - Quick Module Navigation (Priority: P2)

**Goal**: Ensure module cards provide clear descriptions and direct navigation to help users quickly find topics of interest

**Independent Test**: Click each of the 4 module cards and verify: (1) lands on first chapter of that module, (2) can navigate back to homepage and try another card, (3) descriptions on cards accurately reflect module content

### Implementation for User Story 2

- [ ] T024 [US2] Review card descriptions in `book-source/src/components/HomepageFeatures/index.js` for clarity and ensure they match module content from sidebars.js
- [ ] T025 [US2] Verify Module 1 card link `/docs/ros2-fundamentals` matches first item in "Module 1: Robotic Nervous System" from sidebars.js lines 32-38
- [ ] T026 [US2] Verify Module 2 card link `/docs/gazebo-simulation` matches first item in "Module 2: Digital Twin" from sidebars.js lines 40-48
- [ ] T027 [US2] Verify Module 3 card link `/docs/nvidia-isaac-platform` matches first item in "Module 3: AI-Robot Brain" from sidebars.js lines 50-60
- [ ] T028 [US2] Verify Module 4 card link `/docs/introduction-to-vla-systems` matches first item in "Module 4: Vision-Language-Action" from sidebars.js lines 62-72
- [ ] T029 [US2] Test navigation flow: homepage → module 1 → back button → homepage → module 2 (repeat for all 4 modules)
- [ ] T030 [US2] Verify each module landing page content matches the promise in the card description

**Manual Testing Checklist for US2**:
- [ ] Module 1 card navigates to ROS 2 Fundamentals chapter
- [ ] Module 2 card navigates to Gazebo Simulation chapter
- [ ] Module 3 card navigates to NVIDIA Isaac Platform chapter
- [ ] Module 4 card navigates to VLA Systems Introduction chapter
- [ ] Browser back button returns to homepage from any module
- [ ] Card descriptions accurately preview module content
- [ ] Navigation feels intuitive and helpful

**Checkpoint**: At this point, User Stories 1 AND 2 should both work - homepage has clear content AND enables quick module access

---

## Phase 5: User Story 3 - Chatbot Discovery (Priority: P3)

**Goal**: Promote awareness of the AI chatbot feature through the homepage to encourage interactive learning

**Independent Test**: View homepage and verify: (1) 5th card describes chatbot feature, (2) description mentions answering questions with citations, (3) clicking card navigates somewhere reasonable (intro page or activates chatbot if present)

### Implementation for User Story 3

- [ ] T031 [US3] Verify chatbot feature card (card 5) in `book-source/src/components/HomepageFeatures/index.js` has clear description mentioning "questions" and "answers with citations"
- [ ] T032 [US3] Check if chatbot widget is visible on homepage (may appear in sidebar or as floating button)
- [ ] T033 [US3] Test chatbot card navigation: clicking card 5 should go to `/docs/intro` (fallback link since chatbot is global)
- [ ] T034 [US3] Verify chatbot card icon "💬" displays correctly and visually distinguishes this card from module cards
- [ ] T035 [US3] Consider updating card 5 description if actual chatbot behavior differs from description (check README.md for chatbot capabilities)

**Manual Testing Checklist for US3**:
- [ ] Chatbot feature card visible as 5th card
- [ ] Card title "AI Learning Assistant" is clear
- [ ] Description mentions asking questions about textbook content
- [ ] Icon "💬" displays correctly
- [ ] Clicking card navigates to a valid page
- [ ] Users can discover chatbot feature exists from homepage

**Checkpoint**: All user stories (P1, P2, P3) should now be independently functional - complete homepage experience delivered

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Responsive design validation, accessibility testing, and final quality checks

- [ ] T036 [P] Test responsive layout on mobile (375px width) using browser DevTools device toolbar - verify single column layout
- [ ] T037 [P] Test responsive layout on tablet (768px width) using browser DevTools - verify 2-column grid
- [ ] T038 [P] Test responsive layout on desktop (1920px width) - verify 3-column grid with 5 cards wrapping to 2 rows
- [ ] T039 [P] Test very small mobile (320px width) - verify no horizontal scroll and text is readable
- [ ] T040 [P] Test keyboard navigation: Tab through all interactive elements (CTA button, 5 cards) and verify focus indicators visible
- [ ] T041 [P] Test keyboard activation: press Enter on focused CTA button and each card to verify navigation works
- [ ] T042 Run Lighthouse accessibility audit in Chrome DevTools and verify score ≥ 90
- [ ] T043 [P] Check heading hierarchy using browser accessibility inspector: verify h1 (title) → h2 (section) → h3 (cards)
- [ ] T044 [P] Run axe DevTools browser extension and fix any ARIA violations or color contrast issues
- [ ] T045 [P] Test in Firefox browser to verify consistent rendering and navigation
- [ ] T046 [P] Test in Safari browser (if available) to verify emoji icons display correctly
- [ ] T047 Update `book-source/src/components/HomepageFeatures/styles.module.css` if card icon styles need adjustment for emoji (replace `.featureSvg` with `.featureIcon` if needed)
- [ ] T048 Update `book-source/src/pages/index.module.css` if hero section styles need adjustment (typically no changes needed)
- [ ] T049 Run production build using `npm run build` in `book-source/` to verify no broken links
- [ ] T050 Review build output for any warnings or errors related to homepage files
- [ ] T051 Serve production build locally using `npm run serve` in `book-source/` and verify homepage works identically
- [ ] T052 Run Lighthouse performance audit and verify score maintains ≥ 90 (per SC-005)
- [ ] T053 Verify no console errors or React warnings when viewing homepage
- [ ] T054 Take screenshots of homepage at mobile/tablet/desktop widths for documentation
- [ ] T055 Review quickstart.md checklist and verify all 12 validation items pass
- [ ] T056 Clear browser cache and test homepage as a fresh visitor would experience it

**Quality Gates**:
- [ ] All 5 module cards display and navigate correctly
- [ ] "Start Learning" CTA works (no Docusaurus tutorial link)
- [ ] Zero visible "Docusaurus" product references
- [ ] Responsive layout works (320px - 2560px)
- [ ] Keyboard navigation fully functional
- [ ] Lighthouse accessibility ≥ 90
- [ ] Lighthouse performance ≥ 90
- [ ] Production build succeeds
- [ ] No console errors
- [ ] All browsers tested show consistent behavior

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User Story 1 (P1): Can start after Foundational - No dependencies on other stories
  - User Story 2 (P2): Can start after Foundational - Builds on US1 but independently testable
  - User Story 3 (P3): Can start after Foundational - Builds on US1 but independently testable
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
  - Core homepage content replacement
  - Blocks: US2 and US3 (they build on the 5-card structure created here)

- **User Story 2 (P2)**: Can start after US1 implementation
  - Depends on: 5 module cards existing (created in US1)
  - Tests navigation and descriptions of module cards
  - Can be independently validated by testing module links

- **User Story 3 (P3)**: Can start after US1 implementation
  - Depends on: 5-card structure existing (created in US1)
  - Tests chatbot card specifically
  - Can be independently validated by testing chatbot card content and link

**Realistic Execution**: In practice, US1 must complete first (T010-T023) because it creates the 5-card structure that US2 and US3 verify. However, once US1 is done, US2 and US3 can be tested in parallel.

### Within Each User Story

- **User Story 1**: Sequential file editing (T010 → T011 → T012-T016 → T017 → T018-T019 → T020-T023)
  - Hero CTA first (single line change)
  - Then FeatureList content replacement (5 cards)
  - Then Feature component updates to handle new props
  - Then wrapping with Link components
  - Finally testing all navigation

- **User Story 2**: All tasks can run in parallel once US1 completes (T024-T030 are verification tasks)

- **User Story 3**: All tasks can run in parallel once US1 completes (T031-T035 are verification tasks)

### Parallel Opportunities

- **Phase 1 Setup**: T001-T004 are sequential (must install deps before starting server)
- **Phase 2 Foundational**: T005-T009 can all run in parallel (reading different files)
- **Phase 3 User Story 1**:
  - T010 (CTA) can run parallel with T011-T016 (cards) - different line ranges
  - T012-T016 are parallel (editing different cards in same array)
  - T020-T023 are parallel (testing different aspects)
- **Phase 4 User Story 2**: T024-T030 can all run in parallel (independent verification tasks)
- **Phase 5 User Story 3**: T031-T035 can all run in parallel (independent verification tasks)
- **Phase 6 Polish**:
  - T036-T041 can run in parallel (different device/browser tests)
  - T042-T046 can run in parallel (different testing tools)
  - T047-T048 can run in parallel (different CSS files)

---

## Parallel Example: User Story 1 (Core Implementation)

```bash
# These tasks can run in parallel (different line ranges in same file):
Task T010: "Update hero CTA button in book-source/src/pages/index.js line 20-24"
Task T011: "Replace FeatureList array in book-source/src/components/HomepageFeatures/index.js lines 5-36"

# These tasks can run in parallel (editing different cards in array):
Task T012: "Update card 1 content: ROS 2 module"
Task T013: "Update card 2 content: Simulation module"
Task T014: "Update card 3 content: NVIDIA Isaac module"
Task T015: "Update card 4 content: VLA module"
Task T016: "Update card 5 content: Chatbot feature"

# These tasks can run in parallel (testing different aspects):
Task T020: "Verify hot reload shows updated homepage"
Task T021: "Test all 5 card links navigate correctly"
Task T022: "Test Start Learning button navigation"
Task T023: "Search for Docusaurus references"
```

---

## Parallel Example: Phase 6 Polish (Testing)

```bash
# All responsive tests can run in parallel:
Task T036: "Test mobile 375px layout"
Task T037: "Test tablet 768px layout"
Task T038: "Test desktop 1920px layout"
Task T039: "Test mobile 320px minimum width"

# All accessibility tests can run in parallel:
Task T040: "Test keyboard navigation (Tab)"
Task T041: "Test keyboard activation (Enter)"
Task T042: "Run Lighthouse accessibility audit"
Task T043: "Check heading hierarchy"
Task T044: "Run axe DevTools"

# All browser tests can run in parallel:
Task T045: "Test in Firefox"
Task T046: "Test in Safari"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T004) - ~5 minutes
2. Complete Phase 2: Foundational (T005-T009) - ~10 minutes (reading existing code)
3. Complete Phase 3: User Story 1 (T010-T023) - ~30 minutes (core implementation)
4. **STOP and VALIDATE**: Test homepage manually using US1 checklist
5. **Demo Ready**: Homepage now shows textbook content instead of Docusaurus template

**MVP Delivers**: Textbook identity established, clear navigation, professional presentation

### Incremental Delivery

1. **Foundation** (Phase 1-2): Setup + Read existing code → ~15 minutes
2. **MVP** (Phase 3): Add textbook content → Test → ~30 minutes → **Deploy/Demo** ✅
3. **Enhanced Navigation** (Phase 4): Verify module links → Test → ~15 minutes → **Deploy/Demo** ✅
4. **Full Feature Set** (Phase 5): Verify chatbot awareness → Test → ~10 minutes → **Deploy/Demo** ✅
5. **Production Ready** (Phase 6): Polish + accessibility → Test → ~45 minutes → **Deploy/Demo** ✅

**Total Time Estimate**: ~2 hours for complete feature with all polish

### Parallel Team Strategy

With multiple developers:

1. **Team completes Phase 1-2 together** (~15 minutes)
2. **Once Foundational is done:**
   - Developer A: User Story 1 implementation (T010-T019)
   - Developer B: Prepare for US2/US3 testing (read module docs, prepare test cases)
3. **After US1 completes:**
   - Developer A: User Story 2 verification (T024-T030)
   - Developer B: User Story 3 verification (T031-T035)
4. **After US2-US3 complete:**
   - Developer A: Responsive testing (T036-T039)
   - Developer B: Accessibility testing (T040-T046)
   - Developer C: Build and performance testing (T047-T056)

**Parallel Completion Time**: ~1 hour with 2-3 developers

---

## Notes

- **[P] tasks**: Can run in parallel (different files, different test methods, no blocking dependencies)
- **[Story] label**: Maps task to specific user story for traceability
- **Each user story**: Independently completable and testable (US2 and US3 build on US1 structure)
- **Manual testing**: No automated tests - use browser DevTools, Lighthouse, keyboard, and visual validation
- **Hot reload**: Docusaurus dev server auto-reloads on file save for instant feedback
- **Build validation**: `npm run build` catches broken links before deployment
- **Commit strategy**: Commit after each user story phase completes (after T023, T030, T035, T056)
- **Checkpoint validation**: Stop at each phase checkpoint to verify story independently
- **Avoid**: Editing same file lines simultaneously, skipping manual tests, deploying without build validation

---

## Success Criteria Validation

Reference: `specs/001-homepage-redesign/spec.md` Success Criteria

- **SC-001**: First-time visitors identify subject within 5 seconds → Validated by US1 manual test
- **SC-002**: Responsive 320px-2560px without breakage → Validated by Phase 6 tasks T036-T039
- **SC-003**: 1-click navigation to any module → Validated by US2 tasks T025-T029
- **SC-004**: Zero visible "Docusaurus" references → Validated by US1 task T023
- **SC-005**: Maintain ≥90% performance → Validated by Phase 6 task T052
- **SC-006**: WCAG 2.1 Level AA compliance → Validated by Phase 6 tasks T042-T044

All success criteria have corresponding validation tasks in the implementation plan.
