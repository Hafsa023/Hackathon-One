/**
 * Chat Widget Configuration
 *
 * This file contains configuration for the RAG chatbot widget.
 * Update the API_BASE_URL to point to your deployed backend.
 */

export const CHAT_CONFIG = {
  // Backend API URL - Update this for production
  // Development: http://localhost:8000
  // Production: https://your-backend-domain.com
  API_BASE_URL: process.env.NODE_ENV === 'development'
    ? 'http://localhost:8000'
    : 'https://your-backend-domain.com',

  // Chat widget settings
  WIDGET: {
    // Initial collapsed state
    DEFAULT_OPEN: false,

    // Position on screen
    POSITION: {
      bottom: '24px',
      right: '24px',
    },

    // Maximum message history to store locally
    MAX_LOCAL_HISTORY: 50,

    // Minimum selected text length to trigger selection mode
    MIN_SELECTION_LENGTH: 10,

    // Maximum selected text length
    MAX_SELECTION_LENGTH: 5000,
  },

  // Suggested questions shown in empty state
  SUGGESTED_QUESTIONS: [
    'What is embodied intelligence?',
    'How do I set up ROS 2?',
    'Explain sim-to-real transfer',
    'What simulation tools are covered?',
  ],

  // Error messages
  MESSAGES: {
    NETWORK_ERROR: 'Unable to connect to the server. Please try again.',
    GENERIC_ERROR: 'Something went wrong. Please try again.',
    NO_ANSWER: 'I cannot find this information in the book.',
  },
};

export default CHAT_CONFIG;
