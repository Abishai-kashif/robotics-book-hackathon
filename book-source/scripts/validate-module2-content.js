#!/usr/bin/env node

// Comprehensive validation script for Module 2 content structure and quality
// Checks that content follows the required structure and includes necessary elements

const fs = require('fs');
const path = require('path');

// Check if we're in the right directory
const docsPath = path.join(__dirname, '..', 'docs');
const module2Path = path.join(docsPath, 'module2');

if (!fs.existsSync(module2Path)) {
  console.error('Module 2 directory does not exist at expected location');
  process.exit(1);
}

console.log('Validating Module 2 content structure and quality...');

// Check for required files/directories
const requiredFiles = [
  'intro.md',
  'references.md',
  'quality-assurance-checklist.md',
  'guidelines.md'
];

const requiredDirs = [
  'chapter1',
  'chapter2',
  'chapter3'
];

let hasErrors = false;

// Check for required top-level files
for (const file of requiredFiles) {
  const filePath = path.join(module2Path, file);
  if (!fs.existsSync(filePath)) {
    console.error(`Missing required file: ${filePath}`);
    hasErrors = true;
  } else {
    console.log(`✓ Found required file: ${file}`);
  }
}

// Check for required directories
for (const dir of requiredDirs) {
  const dirPath = path.join(module2Path, dir);
  if (!fs.existsSync(dirPath)) {
    console.error(`Missing required directory: ${dirPath}`);
    hasErrors = true;
  } else {
    console.log(`✓ Found required directory: ${dir}`);
  }
}

// Check for required files within chapters
for (const dir of requiredDirs) {
  const dirPath = path.join(module2Path, dir);
  if (fs.existsSync(dirPath)) {
    const files = fs.readdirSync(dirPath);
    const hasIndex = files.includes('index.md');
    const hasExercises = files.includes('exercises.md');

    if (!hasIndex) {
      console.error(`Missing required file index.md in ${dirPath}`);
      hasErrors = true;
    } else {
      console.log(`✓ Found required file: ${path.join(dir, 'index.md')}`);
    }

    if (!hasExercises) {
      console.error(`Missing required file exercises.md in ${dirPath}`);
      hasErrors = true;
    } else {
      console.log(`✓ Found required file: ${path.join(dir, 'exercises.md')}`);
    }
  }
}

// Check for content structure requirements
function validateMarkdownFile(filePath) {
  const content = fs.readFileSync(filePath, 'utf8');

  // Check for required frontmatter
  if (!content.includes('---')) {
    console.error(`Missing frontmatter in ${filePath}`);
    return false;
  }

  // Extract frontmatter - handle both formats (with and without newline after opening ---)
  const frontmatterMatch = content.match(/---\n?([\s\S]*?)\n---/);
  if (!frontmatterMatch) {
    console.error(`Invalid frontmatter format in ${filePath}`);
    return false;
  }

  const frontmatter = frontmatterMatch[1];
  if (!frontmatter.includes('title:')) {
    console.error(`Missing title in frontmatter of ${filePath}`);
    return false;
  }

  if (!frontmatter.includes('sidebar_position:')) {
    console.error(`Missing sidebar_position in frontmatter of ${filePath}`);
    return false;
  }

  // Check for required sections based on file type
  // Chapters need all sections, but special files like references.md have different requirements
  if (filePath.includes('references.md')) {
    // References file should contain references
    if (!content.toLowerCase().includes('references') || !content.includes('# Module 2 References')) {
      console.error(`References file ${filePath} doesn't have proper structure`);
      return false;
    }
  } else if (filePath.includes('exercises.md')) {
    // Exercises files don't need all chapter sections
    if (!content.toLowerCase().includes('exercise')) {
      console.error(`Exercises file ${filePath} should contain exercises`);
      return false;
    }
  } else {
    // Regular chapter files need the standard sections
    const requiredSections = [
      '## Learning Objectives',
      '## Prerequisites',
      '## Summary',
      '## References'
    ];

    for (const section of requiredSections) {
      if (!content.includes(section)) {
        console.error(`Missing required section "${section}" in ${filePath}`);
        return false;
      }
    }
  }

  // Additional quality checks
  // Check for minimum number of learning objectives (only for chapter index files, not exercises)
  if (!filePath.includes('exercises.md')) {
    const objectivesMatch = content.match(/## Learning Objectives[\s\S]*?(- .*\n)+/);
    if (objectivesMatch) {
      const objectives = objectivesMatch[0].match(/- .*/g);
      if (!objectives || objectives.length < 3) {
        console.error(`Chapter ${filePath} should have at least 3 learning objectives`);
        return false;
      }
    }
  }

  // Check for Module 1 connection (with exceptions for special files)
  if (!filePath.includes('exercises.md') && !filePath.includes('references.md')) {
    if (!content.toLowerCase().includes('module 1') && !content.toLowerCase().includes('connection to module 1')) {
      console.warn(`Possible missing Module 1 connection in ${filePath} (not necessarily an error for intro/references)`);
    }
  }

  // Check for academic references (only for chapter index files, not exercises)
  if (filePath.includes('chapter') && !filePath.includes('exercises.md') && !content.toLowerCase().includes('references')) {
    console.error(`Missing references section in chapter file: ${filePath}`);
    return false;
  }

  console.log(`✓ Validated structure for: ${filePath}`);
  return true;
}

// Validate all markdown files in Module 2
function validateDirectory(dirPath) {
  const files = fs.readdirSync(dirPath);
  for (const file of files) {
    const filePath = path.join(dirPath, file);
    if (fs.statSync(filePath).isDirectory()) {
      validateDirectory(filePath);
    } else if (file.endsWith('.md')) {
      // Skip validation for some files that don't follow the standard chapter format
      if (file !== 'quality-assurance-checklist.md' && file !== 'guidelines.md') {
        if (!validateMarkdownFile(filePath)) {
          hasErrors = true;
        }
      } else {
        // Just check basic structure for other markdown files
        const content = fs.readFileSync(filePath, 'utf8');
        if (!content.includes('---')) {
          console.warn(`File ${filePath} doesn't have frontmatter, but that may be intentional`);
        }
        console.log(`✓ Basic check passed for: ${filePath}`);
      }
    }
  }
}

validateDirectory(module2Path);

// Additional checks for cross-module consistency
console.log('\nChecking cross-module consistency...');

// Read all chapter files to check for Module 1 connections
for (const dir of requiredDirs) {
  const dirPath = path.join(module2Path, dir);
  if (fs.existsSync(dirPath)) {
    const indexFile = path.join(dirPath, 'index.md');
    if (fs.existsSync(indexFile)) {
      const content = fs.readFileSync(indexFile, 'utf8');
      if (content.toLowerCase().includes('module 1') || content.toLowerCase().includes('connection to module 1')) {
        console.log(`✓ Found Module 1 connection in ${dir}/index.md`);
      } else {
        console.warn(`⚠️  No obvious Module 1 connection found in ${dir}/index.md`);
      }
    }
  }
}

if (hasErrors) {
  console.error('\n❌ Validation failed with errors');
  process.exit(1);
} else {
  console.log('\n✅ All Module 2 content validations passed');
  process.exit(0);
}