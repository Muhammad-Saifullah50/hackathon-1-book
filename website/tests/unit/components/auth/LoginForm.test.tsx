// website/tests/unit/components/auth/LoginForm.test.tsx
import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import LoginForm from '../../src/components/auth/LoginForm.tsx'; // Updated import
import { AuthProvider, useAuth } from '../../src/hooks/useAuth.tsx'; // Updated import

// Mock the useAuth hook
jest.mock('../../src/hooks/useAuth', () => ({
  ...jest.requireActual('../../src/hooks/useAuth'),
  useAuth: jest.fn(),
}));

const mockLogin = jest.fn();

const renderWithAuth = (ui: React.ReactElement, { providerProps, ...renderOptions }: any) => {
    return render(<AuthProvider {...providerProps}>{ui}</AuthProvider>, renderOptions);
};


describe('LoginForm', () => {
  beforeEach(() => {
    (useAuth as jest.Mock).mockReturnValue({
      login: mockLogin,
      loading: false,
      user: null,
      signup: jest.fn(),
      logout: jest.fn(),
    });
  });

  afterEach(() => {
    jest.clearAllMocks();
  });

  it('renders email and password input fields', () => {
    render(<LoginForm />);
    expect(screen.getByLabelText(/email/i)).toBeInTheDocument();
    expect(screen.getByLabelText(/password/i)).toBeInTheDocument();
  });

  it('calls login with correct credentials on submit', async () => {
    render(<LoginForm />);
    const emailInput = screen.getByLabelText(/email/i);
    const passwordInput = screen.getByLabelText(/password/i);
    const loginButton = screen.getByRole('button', { name: /login/i });

    fireEvent.change(emailInput, { target: { value: 'test@example.com' } });
    fireEvent.change(passwordInput, { target: { value: 'password123' } });
    fireEvent.click(loginButton);

    await waitFor(() => {
      expect(mockLogin).toHaveBeenCalledTimes(1);
      expect(mockLogin).toHaveBeenCalledWith({
        email: 'test@example.com',
        password: 'password123',
      });
    });
  });

  it('shows loading state when logging in', () => {
    (useAuth as jest.Mock).mockReturnValue({
      login: mockLogin,
      loading: true,
      user: null,
      signup: jest.fn(),
      logout: jest.fn(),
    });
    render(<LoginForm />);
    expect(screen.getByRole('button', { name: /logging in/i })).toBeDisabled();
  });

  it('displays error message on failed login', async () => {
    mockLogin.mockRejectedValueOnce(new Error('Login failed')); // Simulate a failed login
    render(<LoginForm />);

    const emailInput = screen.getByLabelText(/email/i);
    const passwordInput = screen.getByLabelText(/password/i);
    const loginButton = screen.getByRole('button', { name: /login/i });

    fireEvent.change(emailInput, { target: { value: 'test@example.com' } });
    fireEvent.change(passwordInput, { target: { value: 'wrongpassword' } });
    fireEvent.click(loginButton);

    await waitFor(() => {
      expect(screen.getByText(/failed to log in/i)).toBeInTheDocument();
    });
  });
});