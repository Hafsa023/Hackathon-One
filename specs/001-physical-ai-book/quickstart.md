# Quickstart: Physical AI Complete Guide Book

**Feature**: 001-physical-ai-book
**Date**: 2025-12-11

## Prerequisites

Before starting, ensure you have:

- **Node.js** 18.x or higher
- **Git** for version control
- **Text editor** with Markdown support (VS Code recommended)
- **Terminal** access (PowerShell on Windows, Bash on Linux/macOS)

## 1. Clone and Setup

```bash
# Clone the repository
git clone <repository-url>
cd Book_writing

# Install Node.js dependencies (when Docusaurus is initialized)
npm install
```

## 2. Project Structure Overview

```
Book_writing/
├── .specify/                    # Spec-Kit Plus configuration
│   ├── memory/
│   │   └── constitution.md      # Project principles
│   ├── templates/               # Document templates
│   └── scripts/                 # Automation scripts
├── specs/
│   └── 001-physical-ai-book/    # Feature specification
│       ├── spec.md              # Requirements
│       ├── plan.md              # Implementation plan
│       ├── research.md          # Technical research
│       ├── data-model.md        # Content entities
│       ├── quickstart.md        # This file
│       ├── contracts/           # Content schemas
│       └── tasks.md             # Implementation tasks (after /sp.tasks)
├── history/
│   ├── prompts/                 # Prompt History Records
│   └── adr/                     # Architecture Decision Records
├── docs/                        # Book content (Docusaurus)
└── static/                      # Images and assets
```

## 3. Development Workflow

### Spec-Kit Plus Commands

```bash
# Create/update feature specification
/sp.specify

# Generate implementation plan
/sp.plan

# Generate task list
/sp.tasks

# Create architecture decision record
/sp.adr <title>

# Record prompt history
/sp.phr
```

### Content Development Cycle

1. **Plan**: Use `/sp.plan` to design chapter structure
2. **Tasks**: Use `/sp.tasks` to break into implementable units
3. **Write**: Create Markdown content in `docs/`
4. **Validate**: Run Docusaurus build to check for errors
5. **Review**: Verify against constitution principles

## 4. Writing a Chapter

### Chapter Template

Create a new chapter file in `docs/`:

```markdown
---
sidebar_position: 1
title: Chapter Title
description: Brief description for SEO
---

# Chapter Title

## Learning Objectives

By the end of this chapter, you will:
- Objective 1
- Objective 2
- Objective 3

## Introduction

[Opening context and motivation]

## Theory / Foundations

[Core concepts and background]

## Architecture / System Design

[System diagrams and component explanations]

## Practical Walkthrough

[Step-by-step implementation guide]

## Code Examples

```python
# Example code with comments
```

## Evaluation / Testing

[How to verify the implementation works]

## Summary

[Key takeaways]

## References

[APA-formatted citations]
```

### Adding Code Examples

Use fenced code blocks with language specification:

````markdown
```python title="publisher_node.py"
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        # Implementation...
```
````

### Adding Diagrams

1. Create diagram in `/static/assets/ch##/`
2. Reference in Markdown:

```markdown
![Architecture diagram](./assets/ch03/ros2-architecture.svg)
```

### Adding Citations

Use consistent APA format:

```markdown
## References

1. Macenski, S., et al. (2022). Robot Operating System 2: Design, Architecture, and Uses in the Wild. *Science Robotics*, 7(66).

2. Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo. *IEEE/RSJ IROS*.
```

## 5. Docusaurus Commands

```bash
# Start development server
npm run start

# Build for production
npm run build

# Deploy to GitHub Pages
npm run deploy

# Check for broken links
npm run docusaurus clear && npm run build
```

## 6. Quality Checklist

Before marking a chapter complete:

- [ ] Word count: 800-2000 words
- [ ] Flesch-Kincaid grade: 8-12
- [ ] Diagrams: 3-7 per chapter
- [ ] Citations: 5+ (at least 1 peer-reviewed)
- [ ] Code examples: Tested and validated
- [ ] Cross-references: All links working
- [ ] Build: Zero errors, zero warnings

## 7. Version Control

### Branch Strategy

```
main                    # Production-ready content
└── 001-physical-ai-book  # Feature development branch
```

### Commit Messages

Follow conventional commits:

```bash
git commit -m "docs(ch03): add ROS 2 node communication examples"
git commit -m "fix(ch04): correct Gazebo plugin configuration"
git commit -m "feat(ch09): implement VLA pipeline walkthrough"
```

## 8. Getting Help

- **Spec-Kit Plus**: Check `.specify/templates/` for document formats
- **Docusaurus**: https://docusaurus.io/docs
- **Constitution**: Review `.specify/memory/constitution.md` for principles
- **PHRs**: Check `history/prompts/` for previous decisions

## Quick Reference

| Task | Command/Location |
|------|------------------|
| Create spec | `/sp.specify` |
| Plan implementation | `/sp.plan` |
| Generate tasks | `/sp.tasks` |
| Write content | `docs/` directory |
| Add images | `static/assets/ch##/` |
| Record decision | `/sp.adr <title>` |
| Check build | `npm run build` |
| Deploy | `npm run deploy` |

---

*Quickstart completed: 2025-12-11*
