import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI',
  tagline: 'A Complete Guide to Embodied Intelligence, Robotics Simulation, and Vision-Language-Action Systems',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-username.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/',

  // GitHub pages deployment config
  organizationName: 'your-username',
  projectName: 'physical-ai-book',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  // Markdown features
  markdown: {
    mermaid: true,
  },

  // Plugins
  themes: ['@docusaurus/theme-mermaid'],

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          routeBasePath: '/docs',
          editUrl: 'https://github.com/your-username/physical-ai-book/tree/main/',
          showLastUpdateTime: false,
          showLastUpdateAuthor: false,
          breadcrumbs: true,
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/physical-ai-social-card.jpg',

    // Color mode configuration
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },

    // Announcement bar (optional)
    announcementBar: {
      id: 'book_progress',
      content: 'This book is actively being developed. Star us on <a target="_blank" rel="noopener noreferrer" href="https://github.com/your-username/physical-ai-book">GitHub</a>!',
      backgroundColor: '#0d9488',
      textColor: '#ffffff',
      isCloseable: true,
    },

    navbar: {
      title: 'Physical AI',
      hideOnScroll: false,
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo-light.svg',
        srcDark: 'img/logo-dark.svg',
        href: '/',
        width: 32,
        height: 32,
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'bookSidebar',
          position: 'left',
          label: 'Book',
        },
        {
          to: '/docs/introduction',
          label: 'Introduction',
          position: 'left',
        },
        {
          to: '/docs/ros2/fundamentals',
          label: 'ROS 2',
          position: 'left',
        },
        {
          to: '/docs/simulation/gazebo',
          label: 'Simulation',
          position: 'left',
        },
        {
          href: 'https://github.com/your-username/physical-ai-book',
          position: 'right',
          className: 'header-github-link',
          'aria-label': 'GitHub repository',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Learn',
          items: [
            {
              label: 'Introduction',
              to: '/docs/introduction',
            },
            {
              label: 'Foundations',
              to: '/docs/foundations/embodied-intelligence',
            },
            {
              label: 'ROS 2 Fundamentals',
              to: '/docs/ros2/fundamentals',
            },
          ],
        },
        {
          title: 'Simulation',
          items: [
            {
              label: 'Gazebo',
              to: '/docs/simulation/gazebo',
            },
            {
              label: 'Unity',
              to: '/docs/simulation/unity',
            },
            {
              label: 'Isaac Sim',
              to: '/docs/simulation/isaac',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'Glossary',
              to: '/docs/appendix/glossary',
            },
            {
              label: 'Installation',
              to: '/docs/appendix/installation',
            },
            {
              label: 'Troubleshooting',
              to: '/docs/appendix/troubleshooting',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/your-username/physical-ai-book',
            },
            {
              label: 'Discussions',
              href: 'https://github.com/your-username/physical-ai-book/discussions',
            },
          ],
        },
      ],
      copyright: `Designed & Built by Hafsa`,
    },

    // Table of contents configuration
    tableOfContents: {
      minHeadingLevel: 2,
      maxHeadingLevel: 4,
    },

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'cpp', 'yaml', 'bash', 'json', 'csharp', 'markup', 'diff', 'toml'],
      magicComments: [
        {
          className: 'theme-code-block-highlighted-line',
          line: 'highlight-next-line',
          block: {start: 'highlight-start', end: 'highlight-end'},
        },
        {
          className: 'code-block-error-line',
          line: 'error-next-line',
        },
      ],
    },

    // Mermaid configuration
    mermaid: {
      theme: {light: 'neutral', dark: 'dark'},
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
