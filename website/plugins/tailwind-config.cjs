// website/plugins/tailwind-config.cjs
function tailwindPlugin(context, options) {
  return {
    name: "tailwind-plugin",
    configurePostCss(postcssOptions) {
      // Directly require the plugins
      const tailwind = require("tailwindcss");
      const autoprefixer = require("autoprefixer");

      postcssOptions.plugins.push(tailwind);
      postcssOptions.plugins.push(autoprefixer);
      return postcssOptions;
    },
  };
}

module.exports = tailwindPlugin;