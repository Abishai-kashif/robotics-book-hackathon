/**
 * Docusaurus plugin for chatbot integration
 */
const path = require('path');

module.exports = function (context, options) {
  return {
    name: 'docusaurus-plugin-chatbot',

    getClientModules() {
      return [path.resolve(__dirname, './src/components/ChatbotWidget.jsx')];
    },

    configureWebpack(config, isServer, utils) {
      return {
        module: {
          rules: [
            {
              test: /\.css$/,
              use: ['style-loader', 'css-loader'],
            },
            {
              test: /\.jsx?$/,
              exclude: /node_modules/,
              use: {
                loader: 'babel-loader',
                options: {
                  presets: ['@babel/preset-env', '@babel/preset-react'],
                },
              },
            },
          ],
        },
      };
    },

    injectHtmlTags() {
      return {
        postBodyTags: [
          `<div id="chatbot-root"></div>`,
        ],
      };
    },
  };
};