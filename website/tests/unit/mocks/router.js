module.exports = {
  useHistory: () => ({
    push: jest.fn(),
    replace: jest.fn(),
    location: { pathname: '/' },
  }),
  useLocation: () => ({
    pathname: '/',
    search: '',
    hash: '',
  }),
};
