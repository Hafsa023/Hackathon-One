import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Physical AI Book Sidebar Configuration
 * Organized by module following the chapter structure from plan.md
 */
const sidebars: SidebarsConfig = {
  bookSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Foundations',
      items: ['foundations/embodied-intelligence'],
    },
    {
      type: 'category',
      label: 'ROS 2',
      items: ['ros2/fundamentals'],
    },
    {
      type: 'category',
      label: 'Simulation',
      items: [
        'simulation/gazebo',
        'simulation/unity',
        'simulation/isaac',
      ],
    },
    {
      type: 'category',
      label: 'Lab Setup',
      items: ['lab/hardware-tools'],
    },
    {
      type: 'category',
      label: 'Sim-to-Real',
      items: ['transfer/sim-to-real'],
    },
    {
      type: 'category',
      label: 'Capstone',
      items: ['capstone/autonomous-humanoid'],
    },
    {
      type: 'category',
      label: 'Appendix',
      items: [
        'appendix/installation',
        'appendix/troubleshooting',
        'appendix/glossary',
        'appendix/compatibility',
      ],
    },
  ],
};

export default sidebars;
