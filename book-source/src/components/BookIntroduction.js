import React from 'react';
import Link from '@docusaurus/Link';

import styles from './BookIntroduction.module.css';

export default function BookIntroduction({
  title = "Physical AI & Humanoid Robotics",
  subtitle = "Bridging the gap between digital AI and physical robots",
  description = "This comprehensive textbook explores the fascinating intersection of artificial intelligence and robotics, focusing on how intelligent systems can interact with the physical world through humanoid robots.",
  level = "College Level",
  subject = "AI & Robotics",
  duration = "One Semester",
  prerequisites = "Calculus, Physics, Programming",
  ctaText = "Start Learning",
  ctaLink = "/docs/intro"
}) {
  return (
    <section className={styles.bookIntroduction}>
      <div className="container">
        <div className="row">
          <div className="col col--12 text--center">
            <div className={styles.educationalBadge}>🎓 Educational Resource</div>
            <span className={styles.academicLevelBadge}>{level}</span>
            <span className={styles.academicLevelBadge}>{subject}</span>
            <h1 className={styles.bookTitle}>{title}</h1>
            <p className={styles.bookSubtitle}>{subtitle}</p>
            <p className={styles.bookDescription}>{description}</p>
            <div className={styles.bookMetadata}>
              <div className={styles.metadataItem}>
                <span className={styles.metadataLabel}>Level</span>
                <span className={styles.metadataValue}>{level}</span>
              </div>
              <div className={styles.metadataItem}>
                <span className={styles.metadataLabel}>Duration</span>
                <span className={styles.metadataValue}>{duration}</span>
              </div>
              <div className={styles.metadataItem}>
                <span className={styles.metadataLabel}>Prerequisites</span>
                <span className={styles.metadataValue}>{prerequisites}</span>
              </div>
              <div className={styles.metadataItem}>
                <span className={styles.metadataLabel}>Subject</span>
                <span className={styles.metadataValue}>{subject}</span>
              </div>
            </div>
            <div className={styles.ctaButton}>
              <Link className="button button--primary button--lg" to={ctaLink}>
                {ctaText}
              </Link>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}