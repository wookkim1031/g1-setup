# Unitree G1 setup notes

How the tools around a Unitree G1 humanoid are set up, one page per setup, written for someone who has
not used them before.

**Read it here: https://wookkim1031.github.io/g1-setup/**

| Page | Status |
|---|---|
| [Basics](https://wookkim1031.github.io/g1-setup/basics.html): LiDAR, IMU, odometry, localisation, TF | Done |
| [Navigation](https://wookkim1031.github.io/g1-setup/navigation.html): LiDAR, DLIO mapping, localisation, Nav2 | Mapping done; the rest in progress |
| [Digital twin](https://wookkim1031.github.io/g1-setup/digital-twin.html): from the map to Isaac Lab and MuJoCo scenes, scanning a chair | Done |

- `docs/`: the pages (GitHub Pages, Jekyll).
- `bin/`: the scripts the pages refer to. `bin/Dockerfile.dlio` builds
  [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry) (feature/ros2) with the settings used for the G1.

The pages describe setup and tools only.
