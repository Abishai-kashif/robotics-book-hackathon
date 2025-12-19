# Data Model: Book Homepage UI

## Book Information Entity
- **name**: String (e.g., "Physical AI & Humanoid Robotics Textbook")
- **subtitle**: String (optional description)
- **author**: String (author name)
- **edition**: String (edition number/version)
- **description**: String (brief overview of the book)
- **learningObjectives**: Array<String> (key objectives of the book)

## Module Entity
- **id**: String (unique identifier like "module-1")
- **title**: String (e.g., "Robotic Nervous System (ROS 2)")
- **description**: String (brief description of the module)
- **position**: Number (order in the book: 1, 2, 3, 4)
- **chapterCount**: Number (number of chapters in the module)
- **learningObjectives**: Array<String> (objectives specific to this module)
- **path**: String (relative path to module content)
- **icon**: String (optional icon identifier for visual representation)

## TableOfContents Entity
- **modules**: Array<Module> (ordered list of all modules)
- **totalChapters**: Number (total chapters in the book)
- **estimatedReadingTime**: Number (estimated time to complete book in hours)

## HomepageLayout Entity
- **heroSection**: Object
  - **title**: String (main headline)
  - **subtitle**: String (subheadline)
  - **ctaText**: String (call to action button text)
  - **ctaLink**: String (link for the call to action)
- **modulesDisplay**: Object
  - **layoutType**: String (grid, list, or card)
  - **showDescriptions**: Boolean (whether to show module descriptions)
  - **showChapterCounts**: Boolean (whether to show chapter counts)
- **tableOfContentsDisplay**: Object
  - **showPreview**: Boolean (whether to show TOC preview)
  - **maxItemsToShow**: Number (maximum items to show in preview)

## NavigationItem Entity
- **id**: String (unique identifier)
- **title**: String (display text)
- **path**: String (relative URL path)
- **type**: String (e.g., "module", "chapter", "section")
- **parent**: String (optional parent item ID)
- **position**: Number (order in navigation)

## LearningObjective Entity
- **id**: String (unique identifier)
- **text**: String (the actual objective statement)
- **module**: String (which module this objective belongs to)
- **priority**: String (P1, P2, P3 for importance)

## ComponentState Entity
- **activeModule**: String (currently selected module ID)
- **showFullTOC**: Boolean (whether full table of contents is expanded)
- **userProgress**: Object (optional user progress tracking)
  - **completedModules**: Array<String> (IDs of completed modules)
  - **currentChapter**: String (currently reading chapter)