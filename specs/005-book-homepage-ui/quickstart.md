# Quickstart: Book Homepage UI

## Overview
This guide will help you implement the book-relevant homepage UI that replaces the default Docusaurus template with an educational-focused layout.

## Prerequisites
- Node.js 18+ installed
- Basic knowledge of React and Docusaurus
- Access to the project repository
- Yarn or npm package manager

## Setup Development Environment

1. **Clone the repository** (if not already done):
```bash
git clone <repository-url>
cd <repository-name>
```

2. **Install dependencies**:
```bash
# Using yarn (recommended)
yarn install

# Or using npm
npm install
```

3. **Start the development server**:
```bash
# Using yarn
yarn start

# Or using npm
npm run start
```

4. **Open your browser** to `http://localhost:3000` to see the current site

## Implementation Steps

### Step 1: Create Custom Homepage Component

1. Create the homepage component file:
```bash
mkdir -p src/pages
touch src/pages/index.js
```

2. Replace the default Docusaurus homepage with the book-focused layout:
```javascript
// src/pages/index.js
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Read the Textbook
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Textbook">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
```

### Step 2: Create Book-Specific Components

1. Create the components directory:
```bash
mkdir -p src/components
```

2. Create a module card component:
```bash
touch src/components/ModuleCard.js
```

3. Create the ModuleCard component:
```javascript
// src/components/ModuleCard.js
import React from 'react';
import Link from '@docusaurus/Link';
import {useBaseUrl} from '@docusaurus/useBaseUrl';

export default function ModuleCard({module}) {
  return (
    <div className="col col--6 margin-bottom--lg">
      <div className="card">
        <div className="card__header">
          <h3>{module.position}. {module.title}</h3>
        </div>
        <div className="card__body">
          <p>{module.description}</p>
          <p><strong>Chapters:</strong> {module.chapterCount}</p>
        </div>
        <div className="card__footer">
          <Link className="button button--primary" to={useBaseUrl(module.path)}>
            Start Learning
          </Link>
        </div>
      </div>
    </div>
  );
}
```

### Step 3: Update Homepage Features Component

1. Create or update the HomepageFeatures component:
```bash
touch src/components/HomepageFeatures.js
```

2. Implement the component to display modules:
```javascript
// src/components/HomepageFeatures.js
import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';
import ModuleCard from './ModuleCard';

const modules = [
  {
    position: 1,
    title: 'Robotic Nervous System (ROS 2)',
    description: 'Introduction to ROS 2 fundamentals and robotic communication systems',
    chapterCount: 4,
    path: '/docs/ros2-fundamentals',
  },
  {
    position: 2,
    title: 'Digital Twin (Gazebo & Unity)',
    description: 'Simulation environments for robotic development and testing',
    chapterCount: 4,
    path: '/docs/gazebo-simulation',
  },
  {
    position: 3,
    title: 'AI-Robot Brain (NVIDIA Isaac™)',
    description: 'AI integration and cognitive systems for robotics',
    chapterCount: 6,
    path: '/docs/nvidia-isaac-platform',
  },
  {
    position: 4,
    title: 'Vision-Language-Action (VLA)',
    description: 'Advanced systems integrating perception, understanding, and action',
    chapterCount: 6,
    path: '/docs/introduction-to-vla-systems',
  },
];

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <h2>Textbook Modules</h2>
            <p>Explore the four comprehensive modules of the Physical AI & Humanoid Robotics textbook:</p>
          </div>
        </div>
        <div className="row">
          {modules.map((module, idx) => (
            <ModuleCard key={idx} module={module} />
          ))}
        </div>
      </div>
    </section>
  );
}
```

### Step 4: Add Custom Styling

1. Create a CSS module for homepage styling:
```bash
touch src/components/HomepageFeatures.module.css
```

2. Add custom styles:
```css
/* src/components/HomepageFeatures.module.css */
.features {
  display: flex;
  align-items: center;
  padding: 2rem 0;
  width: 100%;
}

.featureSvg {
  height: 200px;
  width: 200px;
}
```

### Step 5: Update Docusaurus Configuration

1. Ensure the sidebar configuration supports the new structure
2. Verify navigation links work properly

## Running and Testing

1. **Start the development server**:
```bash
yarn start
```

2. **Verify the new homepage** appears at `http://localhost:3000`
3. **Test navigation** to each module
4. **Check responsive design** on different screen sizes

## Building for Production

```bash
# Build the site
yarn build

# Serve the built site locally for testing
yarn serve
```

## Deployment

The built site can be deployed to any static hosting service. Common options include:
- GitHub Pages
- Netlify
- Vercel
- AWS S3 + CloudFront

## Troubleshooting

- If changes don't appear, restart the development server
- Clear browser cache if styling changes aren't visible
- Check browser console for JavaScript errors
- Verify all module paths exist in the documentation