import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

import styles from './BookHeader.module.css';

export default function BookHeader() {
  const {siteConfig} = useDocusaurusContext();

  return (
    <header className={`hero hero--primary ${styles.bookHeader}`}>
      <div className="container">
        <div className="text--center padding-horiz--md">
          <div className={styles.academicBadge}>🎓 Educational Resource</div>
          <h1 className={`hero__title ${styles.bookTitle}`}>{siteConfig.title}</h1>
          <p className={`hero__subtitle ${styles.bookSubtitle}`}>{siteConfig.tagline}</p>
          <div className={styles.bookMetadata}>
            <span className={styles.metadataItem}>College Level</span>
            <span className={styles.metadataItem}>STEM Education</span>
            <span className={styles.metadataItem}>AI & Robotics</span>
          </div>
          <div className={styles.headerButtons}>
            <Link className="button button--secondary button--lg margin-horiz--sm" to="/docs/intro">
              Start Reading
            </Link>
            <Link className="button button--outline button--lg margin-horiz--sm" to="/docs/intro">
              Table of Contents
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}