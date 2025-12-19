import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import useBaseUrl from '@docusaurus/useBaseUrl';

import { modules, getAllModules } from '../data/modules';
import BookHeader from '../components/BookHeader';
import BookIntroduction from '../components/BookIntroduction';
import ModuleGrid from '../components/ModuleGrid';
import LearningObjectivesPreview from '../components/LearningObjectivesPreview';
import { addSkipLink } from '../utils/accessibility';

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
            Start Reading Textbook
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();

  // Extract all learning objectives from all modules for the preview
  const allLearningObjectives = getAllModules().flatMap(module => module.learningObjectives);

  // Add skip link for accessibility when component mounts
  useEffect(() => {
    addSkipLink();
  }, []);

  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Textbook - Bridging the gap between digital AI and physical robots">
      <BookHeader />
      <main>
        <BookIntroduction />
        <LearningObjectivesPreview objectives={allLearningObjectives} />
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--12">
                <h2>Textbook Modules</h2>
                <p>Explore four comprehensive modules of the Physical AI & Humanoid Robotics textbook:</p>
              </div>
            </div>
          </div>
        </section>
        <ModuleGrid modules={modules} />
      </main>
    </Layout>
  );
}
