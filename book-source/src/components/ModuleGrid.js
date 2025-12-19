import React from 'react';
import ModuleCard from './ModuleCard';

import styles from './ModuleGrid.module.css';

export default function ModuleGrid({ modules }) {
  return (
    <section className={styles.moduleGrid}>
      <div className="container">
        <div className="row">
          {modules.map((module, idx) => (
            <ModuleCard key={idx} module={module} />
          ))}
        </div>
      </div>
    </section>
  );
}