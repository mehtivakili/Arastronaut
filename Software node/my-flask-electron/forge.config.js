module.exports = {
  packagerConfig: {
    asar: {
      unpack: '**/*.node', // Ensures native modules are unpacked
    },
    extraResource: [
      '../server.js',
      '../app.py',
      '../templates',
      '../static',
    ],
  },
  rebuildConfig: {},
  makers: [
    {
      name: '@electron-forge/maker-squirrel',
      config: {
        name: 'ARAStronaut',
        setupExe: "MyARAStronautSetup.exe",
        authors: 'Mehdi',
        description: 'ARASLABS ARAStronaut Application',
      },
    },
    {
      name: '@electron-forge/maker-zip',
    },
    {
      name: '@electron-forge/maker-deb',
    },
    {
      name: '@electron-forge/maker-rpm',
    },
  ],
  plugins: [
    {
      name: '@electron-forge/plugin-auto-unpack-natives',
      config: {
        force: true,
      },
    },
  ],
};
