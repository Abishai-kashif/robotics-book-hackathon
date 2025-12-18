import React from 'react';

// CrossModuleReference component for linking between Module 1 and Module 2 content
const CrossModuleReference = ({ module, chapter, title, description }) => {
  return (
    <div className="cross-module-reference">
      <div className="reference-header">
        <span className="module-label">{module}</span>
        <span className="chapter-label">Chapter: {chapter}</span>
      </div>
      <div className="reference-content">
        <h4>{title}</h4>
        <p>{description}</p>
      </div>
      <div className="reference-link">
        <a href={`../${module.toLowerCase()}/${chapter.toLowerCase()}/`}>
          View {module} Chapter {chapter}
        </a>
      </div>
    </div>
  );
};

export default CrossModuleReference;