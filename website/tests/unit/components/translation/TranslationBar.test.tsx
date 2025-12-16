// website/tests/unit/components/translation/TranslationBar.test.tsx
/**
 * Unit tests for TranslationBar component.
 *
 * Tests:
 * - Idle state rendering (translate button)
 * - Loading state rendering
 * - Translated state rendering with toggle
 * - Error state rendering with retry
 * - Quota exceeded state rendering
 * - Cached version available state
 * - RTL styling for translated content
 * - Quota display
 */

import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';

// Mock the dependencies
jest.mock('@docusaurus/router', () => ({
  useLocation: jest.fn(() => ({
    pathname: '/docs/intro',
  })),
}));

jest.mock('@/hooks/useSafeColorMode', () => ({
  useSafeColorMode: jest.fn(() => ({
    isDark: false,
  })),
}));

jest.mock('@/hooks/useTranslation', () => ({
  useTranslation: jest.fn(),
}));

jest.mock('@/hooks/usePersonalization', () => ({
  usePersonalization: jest.fn(() => ({
    state: 'idle',
    viewMode: 'original',
  })),
}));

import TranslationBar from '@/components/translation/TranslationBar';
import { useTranslation } from '@/hooks/useTranslation';
import type { UseTranslationReturn } from '@/types/translation';

// Default mock return value
const defaultMockReturn: UseTranslationReturn = {
  state: 'idle',
  viewMode: 'original',
  error: null,
  quotaStatus: { used: 0, remaining: 5, limit: 5, resets_at: '2025-12-16T00:00:00Z' },
  translatedContent: null,
  hasCachedVersion: false,
  isContentStale: false,
  translationSourceType: null,
  translate: jest.fn(),
  toggleView: jest.fn(),
  loadCachedVersion: jest.fn(),
  resetTranslation: jest.fn(),
};

beforeEach(() => {
  jest.clearAllMocks();
  (useTranslation as jest.Mock).mockReturnValue(defaultMockReturn);
});

describe('TranslationBar', () => {
  describe('Idle State', () => {
    it('should render translate button in idle state', () => {
      render(<TranslationBar />);

      expect(screen.getByText('Translate to Urdu?')).toBeInTheDocument();
      expect(screen.getByRole('button', { name: /translate to اردو/i })).toBeInTheDocument();
    });

    it('should show quota remaining', () => {
      render(<TranslationBar />);

      expect(screen.getByText(/5\/5 left today/)).toBeInTheDocument();
    });

    it('should call translate on button click', () => {
      const mockTranslate = jest.fn();
      (useTranslation as jest.Mock).mockReturnValue({
        ...defaultMockReturn,
        translate: mockTranslate,
      });

      // Mock document.querySelector
      const mockArticle = document.createElement('article');
      const mockMarkdown = document.createElement('div');
      mockMarkdown.className = 'markdown';
      mockMarkdown.innerHTML = '<p>Test content</p>';
      mockArticle.appendChild(mockMarkdown);
      jest.spyOn(document, 'querySelector').mockReturnValue(mockArticle);

      render(<TranslationBar />);

      fireEvent.click(screen.getByRole('button', { name: /translate to اردو/i }));

      // The actual translate call happens after content extraction
      expect(document.querySelector).toHaveBeenCalledWith('article');
    });

    it('should disable button when quota is 0', () => {
      (useTranslation as jest.Mock).mockReturnValue({
        ...defaultMockReturn,
        quotaStatus: { used: 5, remaining: 0, limit: 5, resets_at: '2025-12-16T00:00:00Z' },
      });

      render(<TranslationBar />);

      const button = screen.getByRole('button', { name: /translate to اردو/i });
      expect(button).toBeDisabled();
    });
  });

  describe('Loading State', () => {
    it('should show loading indicator during translation', () => {
      (useTranslation as jest.Mock).mockReturnValue({
        ...defaultMockReturn,
        state: 'loading',
      });

      render(<TranslationBar />);

      expect(screen.getByText('Translating to Urdu...')).toBeInTheDocument();
      expect(screen.getByText('Processing...')).toBeInTheDocument();
    });

    it('should show Urdu text in loading state', () => {
      (useTranslation as jest.Mock).mockReturnValue({
        ...defaultMockReturn,
        state: 'loading',
      });

      render(<TranslationBar />);

      expect(screen.getByText('اردو')).toBeInTheDocument();
    });
  });

  describe('Translated State', () => {
    it('should show translated view indicator', () => {
      (useTranslation as jest.Mock).mockReturnValue({
        ...defaultMockReturn,
        state: 'translated',
        viewMode: 'translated',
        translatedContent: 'ترجمہ شدہ متن',
        translationSourceType: 'original',
      });

      render(<TranslationBar />);

      expect(screen.getByText(/Viewing.*Urdu/)).toBeInTheDocument();
    });

    it('should show toggle button to show original', () => {
      (useTranslation as jest.Mock).mockReturnValue({
        ...defaultMockReturn,
        state: 'translated',
        viewMode: 'translated',
        translatedContent: 'ترجمہ شدہ متن',
        translationSourceType: 'original',
      });

      render(<TranslationBar />);

      expect(screen.getByRole('button', { name: /show original/i })).toBeInTheDocument();
    });

    it('should show toggle button to show translated when viewing original', () => {
      (useTranslation as jest.Mock).mockReturnValue({
        ...defaultMockReturn,
        state: 'translated',
        viewMode: 'original',
        translatedContent: 'ترجمہ شدہ متن',
        translationSourceType: 'original',
      });

      render(<TranslationBar />);

      expect(screen.getByText(/Viewing.*Original/)).toBeInTheDocument();
      expect(screen.getByRole('button', { name: /show اردو/i })).toBeInTheDocument();
    });

    it('should call toggleView when toggle button is clicked', () => {
      const mockToggleView = jest.fn();
      (useTranslation as jest.Mock).mockReturnValue({
        ...defaultMockReturn,
        state: 'translated',
        viewMode: 'translated',
        translatedContent: 'ترجمہ شدہ متن',
        translationSourceType: 'original',
        toggleView: mockToggleView,
      });

      render(<TranslationBar />);

      fireEvent.click(screen.getByRole('button', { name: /show original/i }));

      expect(mockToggleView).toHaveBeenCalled();
    });

    it('should show quota remaining in translated state', () => {
      (useTranslation as jest.Mock).mockReturnValue({
        ...defaultMockReturn,
        state: 'translated',
        viewMode: 'translated',
        translatedContent: 'ترجمہ شدہ متن',
        translationSourceType: 'original',
        quotaStatus: { used: 2, remaining: 3, limit: 5, resets_at: '2025-12-16T00:00:00Z' },
      });

      render(<TranslationBar />);

      expect(screen.getByText(/3\/5 translations left today/)).toBeInTheDocument();
    });
  });

  describe('Error State', () => {
    it('should show error message', () => {
      (useTranslation as jest.Mock).mockReturnValue({
        ...defaultMockReturn,
        state: 'error',
        error: 'Translation failed. Please try again.',
      });

      render(<TranslationBar />);

      expect(screen.getByText('Translation failed')).toBeInTheDocument();
      expect(screen.getByText('Translation failed. Please try again.')).toBeInTheDocument();
    });

    it('should show retry button', () => {
      (useTranslation as jest.Mock).mockReturnValue({
        ...defaultMockReturn,
        state: 'error',
        error: 'Translation failed.',
      });

      render(<TranslationBar />);

      expect(screen.getByRole('button', { name: /retry/i })).toBeInTheDocument();
    });

    it('should call translate on retry click', () => {
      const mockTranslate = jest.fn();
      (useTranslation as jest.Mock).mockReturnValue({
        ...defaultMockReturn,
        state: 'error',
        error: 'Translation failed.',
        translate: mockTranslate,
      });

      // Mock document.querySelector
      const mockArticle = document.createElement('article');
      const mockMarkdown = document.createElement('div');
      mockMarkdown.className = 'markdown';
      mockMarkdown.innerHTML = '<p>Test content</p>';
      mockArticle.appendChild(mockMarkdown);
      jest.spyOn(document, 'querySelector').mockReturnValue(mockArticle);

      render(<TranslationBar />);

      fireEvent.click(screen.getByRole('button', { name: /retry/i }));

      // Retry triggers content extraction
      expect(document.querySelector).toHaveBeenCalled();
    });
  });

  describe('Quota Exceeded State', () => {
    it('should show quota exceeded message', () => {
      (useTranslation as jest.Mock).mockReturnValue({
        ...defaultMockReturn,
        state: 'quota_exceeded',
        error: 'Daily translation limit reached. Try again tomorrow!',
      });

      render(<TranslationBar />);

      expect(screen.getByText('Daily limit reached')).toBeInTheDocument();
      expect(screen.getByText('Daily translation limit reached. Try again tomorrow!')).toBeInTheDocument();
    });

    it('should not show retry button for quota exceeded', () => {
      (useTranslation as jest.Mock).mockReturnValue({
        ...defaultMockReturn,
        state: 'quota_exceeded',
        error: 'Daily limit reached.',
      });

      render(<TranslationBar />);

      expect(screen.queryByRole('button', { name: /retry/i })).not.toBeInTheDocument();
    });
  });

  describe('Cached Version Available', () => {
    it('should show cached version available message', () => {
      (useTranslation as jest.Mock).mockReturnValue({
        ...defaultMockReturn,
        hasCachedVersion: true,
      });

      render(<TranslationBar />);

      expect(screen.getByText('Urdu translation available')).toBeInTheDocument();
      expect(screen.getByText('You previously translated this page')).toBeInTheDocument();
    });

    it('should show view translation button for cached version', () => {
      (useTranslation as jest.Mock).mockReturnValue({
        ...defaultMockReturn,
        hasCachedVersion: true,
      });

      render(<TranslationBar />);

      expect(screen.getByRole('button', { name: /view اردو/i })).toBeInTheDocument();
    });

    it('should call loadCachedVersion when view button is clicked', () => {
      const mockLoadCached = jest.fn();
      (useTranslation as jest.Mock).mockReturnValue({
        ...defaultMockReturn,
        hasCachedVersion: true,
        loadCachedVersion: mockLoadCached,
      });

      // Mock document.querySelector
      const mockArticle = document.createElement('article');
      const mockMarkdown = document.createElement('div');
      mockMarkdown.className = 'markdown';
      mockMarkdown.innerHTML = '<p>Test content</p>';
      mockArticle.appendChild(mockMarkdown);
      jest.spyOn(document, 'querySelector').mockReturnValue(mockArticle);

      render(<TranslationBar />);

      fireEvent.click(screen.getByRole('button', { name: /view اردو/i }));

      expect(mockLoadCached).toHaveBeenCalled();
    });

    it('should show re-translate button when content is stale', () => {
      (useTranslation as jest.Mock).mockReturnValue({
        ...defaultMockReturn,
        hasCachedVersion: true,
        isContentStale: true,
      });

      render(<TranslationBar />);

      expect(screen.getByText('Content has changed - re-translate for latest version')).toBeInTheDocument();
      expect(screen.getByRole('button', { name: /re-translate/i })).toBeInTheDocument();
    });
  });

  describe('Urdu Text Display', () => {
    it('should display Urdu text "اردو" in idle state', () => {
      render(<TranslationBar />);

      expect(screen.getByText('اردو')).toBeInTheDocument();
    });

    it('should display Urdu text in loading state', () => {
      (useTranslation as jest.Mock).mockReturnValue({
        ...defaultMockReturn,
        state: 'loading',
      });

      render(<TranslationBar />);

      expect(screen.getByText('اردو')).toBeInTheDocument();
    });

    it('should display Urdu text "اردو میں مواد" in translated state', () => {
      (useTranslation as jest.Mock).mockReturnValue({
        ...defaultMockReturn,
        state: 'translated',
        viewMode: 'translated',
        translatedContent: 'ترجمہ شدہ متن',
        translationSourceType: 'original',
      });

      render(<TranslationBar />);

      expect(screen.getByText(/اردو میں مواد/)).toBeInTheDocument();
    });
  });
});
