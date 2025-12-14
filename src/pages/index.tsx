import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

// Floating 3D Robot Component
function Robot3D() {
  return (
    <div className={styles.hero3DContainer}>
      <div className={styles.robot3D}>
        <div className={styles.robotCore}></div>
        <div className={styles.robotRing}></div>
        <div className={styles.robotRing}></div>
        <div className={styles.robotRing}></div>
      </div>
    </div>
  );
}

// Floating Particles
function Particles() {
  const particles = Array.from({ length: 20 }, (_, i) => ({
    id: i,
    left: `${Math.random() * 100}%`,
    delay: `${Math.random() * 10}s`,
    duration: `${8 + Math.random() * 6}s`,
  }));

  return (
    <div className={styles.particles}>
      {particles.map(p => (
        <div
          key={p.id}
          className={styles.particle}
          style={{
            left: p.left,
            animationDelay: p.delay,
            animationDuration: p.duration,
          }}
        />
      ))}
    </div>
  );
}

// Hero Section Component
function HeroSection() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={styles.heroBanner}>
      <div className={styles.heroBackground}>
        <div className={styles.heroGrid}></div>
        <div className={styles.heroGlow}></div>
        <div className={styles.heroGlowSecondary}></div>
        <div className={styles.heroGlowTertiary}></div>
        <Particles />
      </div>
      <Robot3D />
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroBadge}>
            <span className={styles.badgeIcon}>🤖</span>
            <span>The Complete Guide to Embodied Intelligence</span>
          </div>
          <Heading as="h1" className={styles.heroTitle}>
            Master <span className={styles.gradientText}>Physical AI</span>
          </Heading>
          <p className={styles.heroSubtitle}>
            Build intelligent robots that perceive, reason, and act in the real world.
            From ROS 2 fundamentals to Vision-Language-Action models.
          </p>
          <div className={styles.heroButtons}>
            <Link
              className={clsx('button button--lg', styles.primaryButton)}
              to="/docs/foundations/embodied-intelligence">
              Start Learning
            </Link>
            <Link
              className={clsx('button button--lg button--outline', styles.secondaryButton)}
              to="/docs/ros2/fundamentals">
              Explore Chapters
            </Link>
          </div>
          <div className={styles.heroStats}>
            <div className={styles.statItem}>
              <span className={styles.statNumber}>9</span>
              <span className={styles.statLabel}>Chapters</span>
            </div>
            <div className={styles.statDivider}></div>
            <div className={styles.statItem}>
              <span className={styles.statNumber}>3</span>
              <span className={styles.statLabel}>Simulators</span>
            </div>
            <div className={styles.statDivider}></div>
            <div className={styles.statItem}>
              <span className={styles.statNumber}>1</span>
              <span className={styles.statLabel}>Capstone Project</span>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

// Feature Card Data
const FeatureList = [
  {
    title: 'Robotics Foundation',
    icon: '🦾',
    color: 'teal',
    description: 'Master ROS 2 Humble—the industry-standard middleware for robotic systems. Build modular, scalable robot software with publishers, subscribers, and services.',
    badge: 'ROS 2',
  },
  {
    title: 'Simulation Mastery',
    icon: '🎮',
    color: 'violet',
    description: 'Train robots safely in virtual worlds. Learn Gazebo, Unity, and NVIDIA Isaac Sim to create high-fidelity digital twins and accelerate development.',
    badge: 'Multi-Platform',
  },
  {
    title: 'AI Integration',
    icon: '🧠',
    color: 'gradient',
    description: 'Bridge perception and action with Vision-Language-Action models. Deploy AI that understands natural language commands and translates them to robot behaviors.',
    badge: 'VLA Models',
  },
];

function FeatureCard({title, icon, color, description, badge}) {
  return (
    <div className={clsx(styles.featureCard, styles[`featureCard${color}`])}>
      <div className={styles.featureIcon}>{icon}</div>
      <div className={styles.featureBadge}>{badge}</div>
      <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
      <p className={styles.featureDescription}>{description}</p>
    </div>
  );
}

function FeaturesSection() {
  return (
    <section className={styles.featuresSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <span className={styles.sectionLabel}>Core Pillars</span>
          <Heading as="h2" className={styles.sectionTitle}>
            Three Pillars of Physical AI
          </Heading>
          <p className={styles.sectionSubtitle}>
            Physical AI emerges at the intersection of robotics, simulation, and artificial intelligence
          </p>
        </div>
        <div className={styles.featuresGrid}>
          {FeatureList.map((props, idx) => (
            <FeatureCard key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

// Technology Stack Section
const TechStack = [
  { name: 'ROS 2 Humble', logo: '🤖', category: 'Middleware' },
  { name: 'Gazebo Fortress', logo: '🔥', category: 'Simulation' },
  { name: 'Unity', logo: '🎮', category: 'Simulation' },
  { name: 'Isaac Sim', logo: '⚡', category: 'Simulation' },
  { name: 'Python 3.10+', logo: '🐍', category: 'Language' },
  { name: 'PyTorch', logo: '🔦', category: 'ML Framework' },
];

function TechStackSection() {
  return (
    <section className={styles.techSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <span className={styles.sectionLabel}>Technology Stack</span>
          <Heading as="h2" className={styles.sectionTitle}>
            Industry-Standard Tools
          </Heading>
          <p className={styles.sectionSubtitle}>
            Learn the same tools used by leading robotics companies and research labs
          </p>
        </div>
        <div className={styles.techGrid}>
          {TechStack.map((tech, idx) => (
            <div key={idx} className={styles.techCard}>
              <span className={styles.techLogo}>{tech.logo}</span>
              <span className={styles.techName}>{tech.name}</span>
              <span className={styles.techCategory}>{tech.category}</span>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

// Learning Path Section
const LearningPath = [
  {
    step: 1,
    title: 'Foundations',
    chapters: '1-2',
    description: 'Understand embodied intelligence and the theoretical framework behind Physical AI systems.',
    icon: '📚',
  },
  {
    step: 2,
    title: 'ROS 2 Mastery',
    chapters: '3',
    description: 'Build robot nodes, manage communication, and create modular robotic software.',
    icon: '🔧',
  },
  {
    step: 3,
    title: 'Simulation',
    chapters: '4-6',
    description: 'Master Gazebo, Unity, and Isaac Sim to create digital twins and training environments.',
    icon: '🎯',
  },
  {
    step: 4,
    title: 'Sim-to-Real',
    chapters: '7-8',
    description: 'Bridge the reality gap with domain randomization and transfer learning techniques.',
    icon: '🌉',
  },
  {
    step: 5,
    title: 'Capstone',
    chapters: '9',
    description: 'Build an autonomous humanoid robot with voice control and VLA integration.',
    icon: '🏆',
  },
];

function LearningPathSection() {
  return (
    <section className={styles.learningSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <span className={styles.sectionLabel}>Learning Journey</span>
          <Heading as="h2" className={styles.sectionTitle}>
            Your Path to Mastery
          </Heading>
          <p className={styles.sectionSubtitle}>
            A structured curriculum that builds from fundamentals to advanced autonomous systems
          </p>
        </div>
        <div className={styles.pathTimeline}>
          {LearningPath.map((item, idx) => (
            <div key={idx} className={styles.pathItem}>
              <div className={styles.pathConnector}>
                <div className={styles.pathDot}>{item.step}</div>
                {idx < LearningPath.length - 1 && <div className={styles.pathLine}></div>}
              </div>
              <div className={styles.pathContent}>
                <div className={styles.pathIcon}>{item.icon}</div>
                <div className={styles.pathInfo}>
                  <div className={styles.pathHeader}>
                    <Heading as="h4" className={styles.pathTitle}>{item.title}</Heading>
                    <span className={styles.pathChapters}>Ch. {item.chapters}</span>
                  </div>
                  <p className={styles.pathDescription}>{item.description}</p>
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

// What You'll Build Section
const Projects = [
  {
    title: 'ROS 2 Robot Package',
    description: 'Custom nodes with publishers, subscribers, and services',
    icon: '📦',
  },
  {
    title: 'Gazebo Simulation',
    description: 'Robot with sensors in a physics-accurate virtual world',
    icon: '🌍',
  },
  {
    title: 'Isaac Perception Pipeline',
    description: 'GPU-accelerated computer vision for robotics',
    icon: '👁️',
  },
  {
    title: 'Autonomous Humanoid',
    description: 'Voice-controlled robot with VLA integration',
    icon: '🤖',
  },
];

function ProjectsSection() {
  return (
    <section className={styles.projectsSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <span className={styles.sectionLabel}>Hands-On Projects</span>
          <Heading as="h2" className={styles.sectionTitle}>
            What You'll Build
          </Heading>
          <p className={styles.sectionSubtitle}>
            Practical projects that reinforce concepts and build your portfolio
          </p>
        </div>
        <div className={styles.projectsGrid}>
          {Projects.map((project, idx) => (
            <div key={idx} className={styles.projectCard}>
              <span className={styles.projectIcon}>{project.icon}</span>
              <Heading as="h4" className={styles.projectTitle}>{project.title}</Heading>
              <p className={styles.projectDescription}>{project.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

// Prerequisites Section
function PrerequisitesSection() {
  return (
    <section className={styles.prereqSection}>
      <div className="container">
        <div className={styles.prereqGrid}>
          <div className={styles.prereqContent}>
            <span className={styles.sectionLabel}>Prerequisites</span>
            <Heading as="h2" className={styles.sectionTitle}>
              Ready to Start?
            </Heading>
            <p className={styles.prereqDescription}>
              This book targets intermediate-to-advanced learners. Here's what you need:
            </p>
            <ul className={styles.prereqList}>
              <li>
                <span className={styles.prereqIcon}>🐍</span>
                <div>
                  <strong>Python proficiency</strong>
                  <span>Classes, decorators, async programming</span>
                </div>
              </li>
              <li>
                <span className={styles.prereqIcon}>📐</span>
                <div>
                  <strong>Basic linear algebra</strong>
                  <span>Vectors, matrices, transformations</span>
                </div>
              </li>
              <li>
                <span className={styles.prereqIcon}>💻</span>
                <div>
                  <strong>Command-line familiarity</strong>
                  <span>Terminal navigation, package management</span>
                </div>
              </li>
            </ul>
          </div>
          <div className={styles.prereqSpecs}>
            <div className={styles.specCard}>
              <Heading as="h4">Hardware Requirements</Heading>
              <div className={styles.specTiers}>
                <div className={styles.specTier}>
                  <span className={styles.tierLabel}>Minimum</span>
                  <span className={styles.tierSpecs}>16GB RAM, GTX 1660</span>
                  <span className={styles.tierCapability}>ROS 2 + Gazebo</span>
                </div>
                <div className={clsx(styles.specTier, styles.recommended)}>
                  <span className={styles.tierLabel}>Recommended</span>
                  <span className={styles.tierSpecs}>32GB RAM, RTX 3070</span>
                  <span className={styles.tierCapability}>+ Unity + Isaac Sim</span>
                </div>
                <div className={styles.specTier}>
                  <span className={styles.tierLabel}>Optimal</span>
                  <span className={styles.tierSpecs}>64GB RAM, RTX 4080</span>
                  <span className={styles.tierCapability}>Full stack + training</span>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

// CTA Section
function CTASection() {
  return (
    <section className={styles.ctaSection}>
      <div className="container">
        <div className={styles.ctaContent}>
          <Heading as="h2" className={styles.ctaTitle}>
            Ready to Build the Future?
          </Heading>
          <p className={styles.ctaDescription}>
            Join the Physical AI revolution. Start building intelligent robots that perceive,
            reason, and act in the real world.
          </p>
          <div className={styles.ctaButtons}>
            <Link
              className={clsx('button button--lg', styles.ctaPrimary)}
              to="/docs/foundations/embodied-intelligence">
              Start Chapter 1
            </Link>
            <Link
              className={clsx('button button--lg button--outline', styles.ctaSecondary)}
              to="/docs/appendix/installation">
              Setup Guide
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

// Main Homepage Component
export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Home"
      description="A Complete Guide to Embodied Intelligence, Robotics Simulation, and Vision-Language-Action Systems">
      <HeroSection />
      <main>
        <FeaturesSection />
        <TechStackSection />
        <LearningPathSection />
        <ProjectsSection />
        <PrerequisitesSection />
        <CTASection />
      </main>
    </Layout>
  );
}
