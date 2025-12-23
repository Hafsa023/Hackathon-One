# Data Model: Physical AI Complete Guide Book

**Feature**: 001-physical-ai-book
**Date**: 2025-12-11
**Status**: Complete

## Overview

This document defines the entities, relationships, and validation rules for the Physical AI book content management system.

---

## Entity Definitions

### 1. Book

The top-level container for all content.

```yaml
Book:
  attributes:
    title: string (required)
    subtitle: string (optional)
    version: semver (required)
    authors: Author[] (required, min 1)
    created_date: ISO8601 (required)
    updated_date: ISO8601 (required)
    status: enum [draft, review, published] (required)
    word_count_target: range [8000, 20000] (required)

  relationships:
    has_many: Module
    has_many: Chapter
    has_one: Appendix

  validation:
    - title.length <= 100
    - chapters.count >= 8
    - chapters.count <= 12
    - total_word_count >= 8000
    - total_word_count <= 20000
```

### 2. Module

A thematic grouping of chapters covering a major topic area.

```yaml
Module:
  attributes:
    id: string (required, pattern: "module-[0-9]{2}")
    name: string (required)
    description: text (required)
    order: integer (required, min 1)
    learning_objectives: string[] (required, min 2)

  relationships:
    belongs_to: Book
    has_many: Chapter (min 1)

  validation:
    - name.length <= 50
    - description.length <= 500
    - chapters ordered sequentially
```

**Module Instances**:
| ID | Name | Chapters |
|----|------|----------|
| module-01 | Foundations | Ch 1-2 |
| module-02 | ROS 2 Core | Ch 3 |
| module-03 | Simulation | Ch 4-6 |
| module-04 | Infrastructure | Ch 7-8 |
| module-05 | Integration | Ch 9 |

### 3. Chapter

A complete learning unit within the book.

```yaml
Chapter:
  attributes:
    id: string (required, pattern: "ch[0-9]{2}")
    title: string (required)
    slug: string (required, pattern: "[a-z0-9-]+")
    order: integer (required)
    word_count: integer (required, range [800, 2000])
    reading_time_minutes: integer (computed)
    status: enum [outline, draft, review, final] (required)
    flesch_kincaid_grade: float (required, range [8.0, 12.0])

  relationships:
    belongs_to: Module
    has_many: Section (min 3)
    has_many: CodeExample
    has_many: Diagram (min 3, max 7)
    has_many: Citation (min 5)
    has_one: Assessment (optional)

  validation:
    - title.length <= 80
    - sections in standard order
    - at least 1 peer_reviewed citation
    - word_count within range
    - flesch_kincaid_grade within range

  computed:
    reading_time_minutes: ceil(word_count / 200)
```

**Standard Section Order**:
1. Introduction / Overview
2. Theory / Foundations
3. Architecture / System Design
4. Practical Walkthrough
5. Code Examples
6. Evaluation / Testing
7. References

### 4. Section

A subsection within a chapter.

```yaml
Section:
  attributes:
    id: string (required)
    title: string (required)
    order: integer (required)
    type: enum [intro, theory, architecture, walkthrough, code, evaluation, references] (required)
    content: markdown (required)

  relationships:
    belongs_to: Chapter
    has_many: CodeExample
    has_many: Diagram

  validation:
    - title.length <= 60
    - content not empty
```

### 5. CodeExample

A runnable code snippet with metadata.

```yaml
CodeExample:
  attributes:
    id: string (required, pattern: "code-[a-z0-9-]+")
    title: string (required)
    language: enum [python, cpp, yaml, bash, xml, json] (required)
    code: text (required)
    description: text (optional)
    dependencies: string[] (optional)
    expected_output: text (optional)
    validation_status: enum [untested, passing, failing] (required)
    ros_version: string (optional, for ROS examples)
    gazebo_version: string (optional, for Gazebo examples)

  relationships:
    belongs_to: Chapter
    belongs_to: Section (optional)

  validation:
    - code syntax valid for language
    - dependencies list complete
    - version specified if ROS/Gazebo code
```

**Code Example Categories**:
| Language | Primary Use | Chapters |
|----------|-------------|----------|
| Python | rclpy, scripts | 3, 6, 9 |
| C++ | rclcpp, performance | 3, 6 |
| YAML | config, launch | 3, 4, 6 |
| Bash | installation, commands | 3, 4, 7 |
| XML | URDF, SDF | 3, 4 |
| JSON | API responses | 9 |

### 6. Diagram

A visual asset with metadata.

```yaml
Diagram:
  attributes:
    id: string (required, pattern: "fig-[a-z0-9-]+")
    title: string (required)
    alt_text: string (required, accessibility)
    file_path: path (required, pattern: "/static/assets/ch[0-9]{2}/.*")
    file_format: enum [svg, png, webp] (required)
    source_type: enum [ai_generated, self_created, adapted] (required)
    source_attribution: string (required if adapted)
    caption: text (optional)
    width: integer (optional, pixels)

  relationships:
    belongs_to: Chapter
    belongs_to: Section (optional)

  validation:
    - file exists at file_path
    - alt_text.length >= 20
    - alt_text.length <= 200
    - source_attribution required if source_type == adapted
```

**Diagram Categories**:
| Type | Description | Format |
|------|-------------|--------|
| Architecture | System diagrams | SVG |
| Flowchart | Process flows | SVG |
| Screenshot | UI/tool captures | PNG/WebP |
| Schematic | Hardware layouts | SVG |
| Graph | Data visualizations | SVG |

### 7. Citation

A reference to external source material.

```yaml
Citation:
  attributes:
    id: string (required, pattern: "cite-[a-z0-9-]+")
    type: enum [journal, conference, book, documentation, website] (required)
    authors: string[] (required for academic)
    title: string (required)
    year: integer (required)
    venue: string (optional, journal/conference name)
    volume: string (optional)
    pages: string (optional)
    doi: string (optional, pattern: "10\\..*")
    url: url (optional)
    accessed_date: ISO8601 (required if url)
    is_peer_reviewed: boolean (required)
    apa_formatted: string (computed)

  relationships:
    belongs_to: Chapter

  validation:
    - doi or url required
    - url resolves (link check)
    - apa_formatted follows APA 7th edition
```

**Citation Hierarchy** (per Constitution):
1. Official documentation
2. Standards/RFCs
3. Peer-reviewed papers
4. Technical blogs

### 8. Assessment

A practical project or exercise for evaluation.

```yaml
Assessment:
  attributes:
    id: string (required, pattern: "assess-[a-z0-9-]+")
    title: string (required)
    type: enum [project, exercise, quiz] (required)
    difficulty: enum [beginner, intermediate, advanced] (required)
    estimated_time_hours: float (required)
    objectives: string[] (required, min 2)
    deliverables: string[] (required, min 1)
    success_criteria: string[] (required, min 2)
    rubric: RubricItem[] (optional)

  relationships:
    belongs_to: Chapter

  validation:
    - objectives measurable
    - success_criteria testable
```

### 9. HardwareSpec

Equipment requirements for lab setup.

```yaml
HardwareSpec:
  attributes:
    id: string (required)
    category: enum [workstation, edge, sensor, robot, accessory] (required)
    name: string (required)
    specifications: KeyValue[] (required)
    minimum_requirement: boolean (required)
    alternatives: HardwareSpec[] (optional)
    estimated_cost_usd: range (optional)
    purchase_url: url (optional)

  relationships:
    belongs_to: Chapter (Ch 7)

  validation:
    - specifications not empty
    - alternatives have same category
```

---

## Entity Relationships Diagram

```
Book (1)
  │
  ├── has_many ──► Module (4-5)
  │                  │
  │                  └── has_many ──► Chapter (2-3 per module)
  │
  └── has_many ──► Chapter (10)
                     │
                     ├── has_many ──► Section (3-7)
                     │                  │
                     │                  ├── has_many ──► CodeExample
                     │                  └── has_many ──► Diagram
                     │
                     ├── has_many ──► CodeExample (3-10)
                     ├── has_many ──► Diagram (3-7)
                     ├── has_many ──► Citation (5+)
                     └── has_one ───► Assessment (optional)
```

---

## State Transitions

### Chapter Status Flow

```
outline ──► draft ──► review ──► final
   │          │          │
   └──────────┴──────────┴──► (can revert to previous state)
```

**Transition Rules**:
| From | To | Requirements |
|------|----|--------------|
| outline | draft | All sections defined, structure complete |
| draft | review | Word count met, code examples added, diagrams placed |
| review | final | FK grade verified, citations complete, peer reviewed |

### CodeExample Validation Flow

```
untested ──► passing
    │           │
    └───────────┴──► failing ──► passing
```

---

## Validation Rules Summary

### Book-Level
- [ ] Total word count: 8,000-20,000
- [ ] Chapter count: 10 (+ appendix)
- [ ] All chapters have status=final before publish

### Chapter-Level
- [ ] Word count: 800-2,000
- [ ] Flesch-Kincaid grade: 8-12
- [ ] Diagrams: 3-7 per chapter
- [ ] Citations: 5+ per chapter (1+ peer-reviewed)
- [ ] Standard section structure followed

### Code-Level
- [ ] All code examples pass syntax validation
- [ ] Dependencies documented
- [ ] Version requirements specified

### Citation-Level
- [ ] All URLs resolve (link check)
- [ ] DOIs valid where provided
- [ ] APA 7th edition format

---

## Indexes and Queries

### Primary Indexes
- `chapters_by_module`: Chapter[] grouped by module_id
- `code_examples_by_language`: CodeExample[] grouped by language
- `citations_by_type`: Citation[] grouped by type

### Common Queries
```sql
-- Get all incomplete chapters
SELECT * FROM chapters WHERE status != 'final'

-- Get chapters below word count
SELECT * FROM chapters WHERE word_count < 800

-- Get chapters missing peer-reviewed citations
SELECT c.* FROM chapters c
WHERE NOT EXISTS (
  SELECT 1 FROM citations ct
  WHERE ct.chapter_id = c.id AND ct.is_peer_reviewed = true
)

-- Get untested code examples
SELECT * FROM code_examples WHERE validation_status = 'untested'
```

---

*Data model completed: 2025-12-11*
