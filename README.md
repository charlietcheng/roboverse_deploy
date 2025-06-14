# :robot: Roboverse Deploy

Deploy your sim-to-real robot policies trained on [RoboVerse](https://github.com/RoboVerseOrg/RoboVerse) to real-world robotic platforms! :rocket:

## :dart: Project Goal

roboverse_deploy bridges the gap between simulation and reality by providing deployment infrastructure for policies trained on the RoboVerse dataset and benchmark platform. This repository enables researchers and developers to seamlessly transfer their large-scale robot policies from simulation to various real-world robotic embodiments.

### :star2: Key Features
- **Multi-Robot Support**: Each folder represents a different robotic platform with its own deployment infrastructure
- **Sim2Real Ready**: Designed specifically for deploying policies trained in Roboverse simulation
- **Modular Architecture**: Easy to extend with new robot platforms
- **Docker-based Deployment**: Consistent environment across different systems

## :robot: Supported Platforms

- **:white_check_mark: Franka Research 3 (FR3)** - Fully implemented with both ROS1 and ROS2 integration
- **:arrows_counterclockwise: Unitree Robots** - Coming soon
- **:arrows_counterclockwise: AllegroHand** - Coming soon
- **:arrows_counterclockwise: ARX7** - Coming soon  
- **:arrows_counterclockwise: Booster T1** - Coming soon

## :rocket: Example Usage

check out README files in each folder for more.

## :clipboard: Future Plans

### Infrastructure
- [ ] Support Unitree G1 manipulation tasks
- [ ] Support Unitree G1 locomotion tasks
- [ ] Support Booster T1 locomotion tasks
- [ ] Support AllegroHand
- [ ] Franka with different end effectors
- [ ] Multi-robot coordination

## :hammer_and_wrench: Architecture Overview

```
roboverse_deploy/
├── :mechanical_arm: franka_research_3_ros/    # Franka arm deployment
├── :dog: unitree_python/           # Unitree quadrupeds/humanoids  
├── :raised_hand: allegrohand/              # Dexterous hand control
├── :mechanical_arm: arx7/                     # ARX7 arm platform
└── :robot: booster_t1/               # Booster T1 robot
```

Each platform folder contains:
- Docker configuration for consistent deployment
- ROS/SDK integration
- Policy loading and execution
- Platform-specific utilities

## :handshake: Contributing

We welcome contributions! Whether you're adding support for new robot platforms, improving existing infrastructure, or enhancing sim2real transfer capabilities, your input helps the entire robotics community. :earth_americas:

## :books: Learn More

- [Roboverse Dataset & Benchmark](https://github.com/roboverse/roboverse)
- [Documentation](./docs) (Coming soon)
- [Example Policy Checkpoints](./examples) (Coming soon)
