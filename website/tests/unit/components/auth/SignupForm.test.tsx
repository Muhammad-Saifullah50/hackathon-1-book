// website/tests/unit/components/auth/SignupForm.test.tsx
import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import SignupForm from '@/components/auth/SignupForm'; // Updated import
import { useAuth } from '@/hooks/useAuth'; // Updated import
import { useHistory } from '@docusaurus/router';

// Mock the useAuth hook
jest.mock('@/hooks/useAuth', () => ({
  useAuth: jest.fn(),
}));

// Mock useHistory
jest.mock('@docusaurus/router', () => ({
  useHistory: jest.fn(),
}));

const mockSignup = jest.fn();
const mockHistoryPush = jest.fn();

describe('SignupForm', () => {
  beforeEach(() => {
    (useAuth as jest.Mock).mockReturnValue({
      signup: mockSignup,
      loading: false,
      error: null,
      token: null, // Default to no token
    });
    (useHistory as jest.Mock).mockReturnValue({
      push: mockHistoryPush,
    });
    localStorage.clear();
  });

  afterEach(() => {
    jest.clearAllMocks();
  });

  it('renders email and password input fields', () => {
    render(<SignupForm />);
    expect(screen.getByLabelText(/email/i)).toBeInTheDocument();
    expect(screen.getByLabelText(/password/i)).toBeInTheDocument();
  });

  it('calls signup and shows verification message if no token is returned', async () => {
    mockSignup.mockResolvedValue(null); // Success
    render(<SignupForm />);
    const emailInput = screen.getByLabelText(/email/i);
    const passwordInput = screen.getByLabelText(/password/i);
    const signupButton = screen.getByRole('button', { name: /sign up/i });

    fireEvent.change(emailInput, { target: { value: 'newuser@example.com' } });
    fireEvent.change(passwordInput, { target: { value: 'newpassword123' } });
    fireEvent.click(signupButton);

    await waitFor(() => {
      expect(mockSignup).toHaveBeenCalledTimes(1);
      expect(screen.getByText(/please check your email/i)).toBeInTheDocument();
      expect(mockHistoryPush).not.toHaveBeenCalled();
    });
  });

  it('calls signup and redirects if token is present (auto-login)', async () => {
    mockSignup.mockResolvedValue(null);
    localStorage.setItem('access_token', 'fake-token'); // Simulate token being set

    render(<SignupForm />);
    const emailInput = screen.getByLabelText(/email/i);
    const passwordInput = screen.getByLabelText(/password/i);
    const signupButton = screen.getByRole('button', { name: /sign up/i });

    fireEvent.change(emailInput, { target: { value: 'newuser@example.com' } });
    fireEvent.change(passwordInput, { target: { value: 'newpassword123' } });
    fireEvent.click(signupButton);

    await waitFor(() => {
      expect(mockHistoryPush).toHaveBeenCalledWith('/signup-wizard');
    });
  });

  it('shows loading state when signing up', () => {
    (useAuth as jest.Mock).mockReturnValue({
      signup: mockSignup,
      loading: true,
      error: null,
      token: null,
    });
    render(<SignupForm />);
    expect(screen.getByRole('button', { name: /signing up/i })).toBeDisabled();
  });

  it('displays error message on failed signup', async () => {
    const errorMessage = 'Email already exists';
    mockSignup.mockResolvedValue(errorMessage); 
    
    render(<SignupForm />);

    const emailInput = screen.getByLabelText(/email/i);
    const passwordInput = screen.getByLabelText(/password/i);
    const signupButton = screen.getByRole('button', { name: /sign up/i });

    fireEvent.change(emailInput, { target: { value: 'fail@example.com' } });
    fireEvent.change(passwordInput, { target: { value: 'fakepassword' } });
    fireEvent.click(signupButton);

    await waitFor(() => {
        expect(screen.getByText(errorMessage)).toBeInTheDocument();
        expect(mockHistoryPush).not.toHaveBeenCalled();
    });
  });
});