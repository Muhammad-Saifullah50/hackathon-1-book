// website/jest.config.js
module.exports = {
  testEnvironment: 'jsdom',
  transform: {
    '^.+\.(ts|tsx)$': 'ts-jest',
    '^.+\.(js|jsx)$': 'babel-jest',
  },
  moduleNameMapper: {
    '^@/(.*)$': '<rootDir>/src/$1', // Map @ to the src directory
    '^@site/(.*)$': '<rootDir>/$1', // Map @site to root directory
    '@docusaurus/useDocusaurusContext': '<rootDir>/tests/unit/mocks/useDocusaurusContext.js',
    '@docusaurus/Link': '<rootDir>/tests/unit/mocks/Link.js',
    '@docusaurus/router': '<rootDir>/tests/unit/mocks/router.js',
    '\\.css$': '<rootDir>/tests/unit/mocks/styleMock.js',
  },
  setupFilesAfterEnv: ['<rootDir>/setupTests.ts'],
  testMatch: ['<rootDir>/src/**/*.{spec,test}.{js,jsx,ts,tsx}', '<rootDir>/tests/unit/**/*.{spec,test}.{js,jsx,ts,tsx}'],
};
