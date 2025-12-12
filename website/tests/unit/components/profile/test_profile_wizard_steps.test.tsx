// website/tests/unit/components/profile/test_profile_wizard_steps.test.tsx

import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import ProfileWizardStep1 from '@/components/profile/ProfileWizardStep1';
import ProfileWizardStep2 from '@/components/profile/ProfileWizardStep2';
import ProfileWizardStep3 from '@/components/profile/ProfileWizardStep3';
import { UserProfileUpdate } from '@/data/profile-schema';

// Mock the uuidv4 import if necessary for components that use it directly
jest.mock('uuid', () => ({
  v4: () => 'mock-uuid',
}));

describe('ProfileWizardStep1', () => {
  const mockOnNext = jest.fn();
  const initialData: Partial<UserProfileUpdate> = {};

  
  beforeEach(() => {
    mockOnNext.mockClear();
  });

  it('renders correctly and calls onNext with selected age range', async () => {
    render(<ProfileWizardStep1 onNext={mockOnNext} initialData={initialData} />);

    expect(screen.getByText('Step 1: The Basics')).toBeInTheDocument();
    
    const select = screen.getByLabelText(/age range/i);
    fireEvent.change(select, { target: { value: '18_24' } });
    
    fireEvent.click(screen.getByRole('button', { name: /next/i }));

    await waitFor(() => {
      expect(mockOnNext).toHaveBeenCalledWith({ age_range: '18_24' });
    });
  });
});

describe('ProfileWizardStep2', () => {
  const mockOnNext = jest.fn();
  const mockOnBack = jest.fn();
  const initialData: Partial<UserProfileUpdate> = { age_range: '18_24' };

  beforeEach(() => {
    mockOnNext.mockClear();
    mockOnBack.mockClear();
  });

  it('renders correctly and calls onNext with selected data', async () => {
    render(<ProfileWizardStep2 onNext={mockOnNext} onBack={mockOnBack} initialData={initialData} />);

    expect(screen.getByText('Step 2: The Background')).toBeInTheDocument();
    
    const educationSelect = screen.getByLabelText(/education level/i);
    fireEvent.change(educationSelect, { target: { value: 'undergrad' } });

    const techBackgroundSelect = screen.getByLabelText(/tech background/i);
    fireEvent.change(techBackgroundSelect, { target: { value: 'student' } });

    fireEvent.click(screen.getByRole('button', { name: /next/i }));

    await waitFor(() => {
      expect(mockOnNext).toHaveBeenCalledWith({
        education_level: 'undergrad',
        tech_background: 'student',
      });
    });
  });

  it('calls onBack when back button is clicked', async () => {
    render(<ProfileWizardStep2 onNext={mockOnNext} onBack={mockOnBack} initialData={initialData} />);
    fireEvent.click(screen.getByRole('button', { name: /back/i }));
    expect(mockOnBack).toHaveBeenCalledTimes(1);
  });
});

describe('ProfileWizardStep3', () => {
  const mockOnSubmit = jest.fn();
  const mockOnBack = jest.fn();
  const initialData: Partial<UserProfileUpdate> = {
    age_range: '18_24',
    education_level: 'undergrad',
    tech_background: 'student',
  };

  beforeEach(() => {
    mockOnSubmit.mockClear();
    mockOnBack.mockClear();
  });

  it('renders correctly and calls onSubmit with all data', async () => {
    render(<ProfileWizardStep3 onSubmit={mockOnSubmit} onBack={mockOnBack} initialData={initialData} />);

    expect(screen.getByText('Step 3: The Strategy')).toBeInTheDocument();

    const goalSelect = screen.getByLabelText(/primary goal/i);
    fireEvent.change(goalSelect, { target: { value: 'academic' } });

    const modeSelect = screen.getByLabelText(/learning mode/i);
    fireEvent.change(modeSelect, { target: { value: 'textual' } });

    const speedSelect = screen.getByLabelText(/learning speed/i);
    fireEvent.change(speedSelect, { target: { value: 'balanced' } });

    const timeInput = screen.getByLabelText(/time per week/i);
    fireEvent.change(timeInput, { target: { value: '10' } });

    fireEvent.click(screen.getByRole('button', { name: /finish & create profile/i }));

    await waitFor(() => {
      expect(mockOnSubmit).toHaveBeenCalledWith({
        primary_goal: 'academic',
        learning_mode: 'textual',
        learning_speed: 'balanced',
        time_per_week: 10,
      });
    });
  });

  it('calls onBack when back button is clicked', async () => {
    render(<ProfileWizardStep3 onSubmit={mockOnSubmit} onBack={mockOnBack} initialData={initialData} />);
    fireEvent.click(screen.getByRole('button', { name: /back/i }));
    expect(mockOnBack).toHaveBeenCalledTimes(1);
  });
});