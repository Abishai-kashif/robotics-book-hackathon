import React from 'react';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';

import styles from './ModuleCard.module.css';

export default function ModuleCard({module}) {
  return (
    <div className={`col col--6 margin-bottom--lg ${styles.moduleCard}`}>
      <div className="card">
        <div className="card__header">
          <h3>{module.position}. {module.title}</h3>
        </div>
        <div className="card__body">
          <div className={styles.moduleBadge}>Module {module.position}</div>
          {module.difficulty && (
            <div className={styles.moduleDifficulty}>{module.difficulty}</div>
          )}
          {module.duration && (
            <div className={styles.moduleDuration}>{module.duration}</div>
          )}
          {module.prerequisites && (
            <div className={styles.modulePrerequisites}>{module.prerequisites}</div>
          )}
          <p className={styles.academicModuleDescription}>{module.description}</p>
          <p><strong>Chapters:</strong> {module.chapterCount}</p>
          {module.learningObjectives && module.learningObjectives.length > 0 && (
            <div>
              <strong>Learning Objectives:</strong>
              <ul className={styles.learningObjectivesList}>
                {module.learningObjectives.slice(0, 3).map((objective, index) => (
                  <li key={index}>{objective}</li>
                ))}
              </ul>
            </div>
          )}
        </div>
        <div className="card__footer">
          <Link className="button button--primary" to={useBaseUrl(module.path)}>
            Explore Module
          </Link>
        </div>
      </div>
    </div>
  );
}