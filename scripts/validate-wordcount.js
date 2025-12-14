#!/usr/bin/env node
/**
 * Word Count Validation Script for Physical AI Book
 * Validates that each chapter meets the 800-2000 word requirement (FR-001)
 *
 * Usage: node scripts/validate-wordcount.js [--verbose]
 */

const fs = require('fs');
const path = require('path');

// Configuration
const DOCS_DIR = path.join(__dirname, '..', 'docs');
const MIN_WORDS = 800;
const MAX_WORDS = 2000;
const TOTAL_MIN = 8000;
const TOTAL_MAX = 20000;

// Chapter files to validate (excluding appendix content from word count)
const CHAPTER_PATTERNS = [
  'intro.md',
  'foundations/embodied-intelligence.md',
  'ros2/fundamentals.md',
  'simulation/gazebo.md',
  'simulation/unity.md',
  'simulation/isaac.md',
  'lab/hardware-tools.md',
  'transfer/sim-to-real.md',
  'capstone/autonomous-humanoid.md',
];

/**
 * Strip markdown formatting to get plain text
 */
function stripMarkdown(content) {
  return content
    // Remove frontmatter
    .replace(/^---[\s\S]*?---/m, '')
    // Remove code blocks
    .replace(/```[\s\S]*?```/g, '')
    // Remove inline code
    .replace(/`[^`]+`/g, '')
    // Remove images
    .replace(/!\[.*?\]\(.*?\)/g, '')
    // Remove links but keep text
    .replace(/\[([^\]]+)\]\([^)]+\)/g, '$1')
    // Remove HTML tags
    .replace(/<[^>]+>/g, '')
    // Remove headers markers
    .replace(/^#+\s*/gm, '')
    // Remove emphasis markers
    .replace(/[*_]{1,2}([^*_]+)[*_]{1,2}/g, '$1')
    // Remove blockquotes
    .replace(/^>\s*/gm, '')
    // Remove horizontal rules
    .replace(/^[-*_]{3,}$/gm, '')
    // Remove list markers
    .replace(/^[\s]*[-*+]\s*/gm, '')
    .replace(/^[\s]*\d+\.\s*/gm, '')
    // Remove extra whitespace
    .replace(/\s+/g, ' ')
    .trim();
}

/**
 * Count words in text
 */
function countWords(text) {
  const plainText = stripMarkdown(text);
  if (!plainText) return 0;
  return plainText.split(/\s+/).filter(word => word.length > 0).length;
}

/**
 * Validate a single chapter file
 */
function validateChapter(filePath) {
  const fullPath = path.join(DOCS_DIR, filePath);

  if (!fs.existsSync(fullPath)) {
    return {
      file: filePath,
      exists: false,
      wordCount: 0,
      valid: false,
      message: 'File not found',
    };
  }

  const content = fs.readFileSync(fullPath, 'utf-8');
  const wordCount = countWords(content);
  const valid = wordCount >= MIN_WORDS && wordCount <= MAX_WORDS;

  let message = '';
  if (wordCount < MIN_WORDS) {
    message = `Below minimum (${MIN_WORDS - wordCount} words short)`;
  } else if (wordCount > MAX_WORDS) {
    message = `Above maximum (${wordCount - MAX_WORDS} words over)`;
  } else {
    message = 'OK';
  }

  return {
    file: filePath,
    exists: true,
    wordCount,
    valid,
    message,
  };
}

/**
 * Main validation function
 */
function main() {
  const verbose = process.argv.includes('--verbose');
  const results = [];
  let totalWords = 0;
  let validCount = 0;
  let existingCount = 0;

  console.log('\\n📚 Physical AI Book - Word Count Validation');
  console.log('='.repeat(60));
  console.log(`Requirements: ${MIN_WORDS}-${MAX_WORDS} words per chapter`);
  console.log(`Total book: ${TOTAL_MIN}-${TOTAL_MAX} words\\n`);

  // Validate each chapter
  for (const chapter of CHAPTER_PATTERNS) {
    const result = validateChapter(chapter);
    results.push(result);

    if (result.exists) {
      existingCount++;
      totalWords += result.wordCount;
      if (result.valid) validCount++;
    }

    const status = result.exists
      ? (result.valid ? '✅' : '❌')
      : '⚠️';

    console.log(
      `${status} ${result.file.padEnd(40)} ${String(result.wordCount).padStart(5)} words  ${result.message}`
    );
  }

  // Summary
  console.log('\\n' + '='.repeat(60));
  console.log('SUMMARY');
  console.log('='.repeat(60));
  console.log(`Chapters found: ${existingCount}/${CHAPTER_PATTERNS.length}`);
  console.log(`Chapters valid: ${validCount}/${existingCount}`);
  console.log(`Total words: ${totalWords}`);

  const totalValid = totalWords >= TOTAL_MIN && totalWords <= TOTAL_MAX;
  console.log(
    `Total requirement: ${totalValid ? '✅' : '❌'} (${TOTAL_MIN}-${TOTAL_MAX})`
  );

  // Verbose output
  if (verbose) {
    console.log('\\nDetailed Results:');
    console.log(JSON.stringify(results, null, 2));
  }

  // Exit code
  const allValid = validCount === existingCount && totalValid;
  process.exit(allValid ? 0 : 1);
}

// Run if called directly
if (require.main === module) {
  main();
}

module.exports = { countWords, stripMarkdown, validateChapter };
