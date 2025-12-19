import React from 'react';

import styles from './LearningObjectivesPreview.module.css';

export default function LearningObjectivesPreview({ objectives = [], title = "Learning Objectives" }) {
  return (
    <section className={styles.learningObjectivesPreview}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <h2>{title}</h2>
            <ul className={styles.objectivesList}>
              {objectives.map((objective, index) => (
                <li key={index} className={styles.objectiveItem}>
                  <div className={styles.objectiveIcon}>📚</div>
                  <div className={styles.objectiveText}>{objective}</div>
                </li>
              ))}
            </ul>
          </div>
        </div>
      </div>
    </section>
  );
}