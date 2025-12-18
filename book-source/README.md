# Physical AI & Humanoid Robotics Textbook - Website

This website is built using [Docusaurus](https://docusaurus.io/), a modern static website generator.

This repository contains the Physical AI & Humanoid Robotics textbook, with content organized into modules:

- **Module 1**: Robotic Nervous System (ROS 2)
- **Module 2**: Digital Twin (Gazebo & Unity) - *Newly added*
- **Module 3**: AI-Robot Brain (NVIDIA Isaac™)
- **Module 4**: Vision-Language-Action (VLA)
- **Module 5**: Capstone: Autonomous Humanoid

## Installation

```bash
yarn
```

## Local Development

```bash
yarn start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
yarn build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true yarn deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> yarn deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.
