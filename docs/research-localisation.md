---
layout: default
title: Localisation and maps
---

[Back to the research overview](research.html)

# Localisation and maps: knowing where the robot is

To walk to a place on a map, the robot needs three things: a smooth estimate of its own motion
(odometry), its position on the stored map (localisation), and a map of where it may walk. This page lists
the options for each, for a walking robot with a Livox Mid-360 LiDAR in its head.

## Odometry from the LiDAR

Several open programs turn LiDAR and motion-sensor data into the robot's motion. We use DLIO.

| Program | In short |
|---|---|
| **DLIO** | What we use. Its paper reports less computing and better accuracy than earlier methods [1] |
| **FAST-LIO2** | Used as the baseline in the legged-robot paper below |
| **KISS-ICP** | Was the least steady of the three in the test below |
| **GLIM** | Uses a graphics card and corrects its path when it returns to a known place |
| **Point-LIO, LIO-SAM, iG-LIO, FAST-LIVO2** | Further programs named in comparisons; not looked at in detail |

There is **no clear winner**. One test on the same LiDAR and the same class of onboard computer as ours,
on seven indoor drone flights, measured an average error of about 10 cm for DLIO, 11 cm for FAST-LIO2 and
12 cm for KISS-ICP, with GLIM at 4 cm on a graphics card [2]. It is a single forum post about a drone,
not a walking robot. On other data sets the order changes.

For walking robots, which shake at every step, the evidence is thin:

* one method adds the legs' own motion to LiDAR and motion sensor on a two-legged robot [3];
* one models how vibration blurs each scan and weights the points accordingly [4].

We found no comparison of these programs on a humanoid with our LiDAR.

**The robot maker's own service.** Unitree's documentation lists a mapping and navigation service for its
robots, with commands to start and stop mapping, set the start position and walk to a pose [5]. We could
not read the page itself, only a copy, so the details are unconfirmed. Our LiDAR recorder already listens
for that service's position messages and has received none so far.

## Finding the robot on the stored map

Odometry starts at zero every time. Localisation matches the current scan against the stored map.

| Approach | Needs a start position? | Notes |
|---|---|---|
| **Scan-to-map tracking** (NDT, GICP, small_gicp) | Yes, roughly right | Fast and accurate once started. Open ROS 2 packages exist. Fails if the start is far off |
| **Global registration** (KISS-Matcher) | **No** | Finds the pose from one scan. With a fine-matching step afterwards, its error fell from about 18 cm to about 1 cm, measured on outdoor driving data [6] |
| **Place descriptors** (Scan Context and others) | No | Recognise a place by a compact signature of the scan |
| **2D localisation on a flattened scan** (AMCL) | Yes | The classic for wheeled robots; throws away the height information |
| **Printed markers** | No | What Spot uses to start: stand near a marker seen during recording |

A described recipe combines two of these: a global match for the first position, then fast tracking, with
exactly one program publishing the result. Two programs publishing it at once make the position jump [7].
That description is a blog post without measurements.

Our own simulation had found that tracking survives moved furniture but cannot recover from a start more
than about half a metre and ten degrees off. A global matcher removes that limit.

## Maps that stay valid

A stored map goes out of date when furniture moves, and it contains whatever stood there during the scan.

| Problem | What exists |
|---|---|
| **People and moved things in the map** | Programs that remove moving points from a recorded map: Removert, ERASOR, Dynablox, DUFOMap. A comparison found each trades off differently between keeping the fixed structure and removing what moved [8]. Tested mostly on outdoor driving data |
| **Glass and reflections** | Research that detects reflecting planes and removes the ghost points behind them. Not tested on our kind of LiDAR |
| **A changing room** | Keep only what appears in several scans taken on different days, which is what we did by hand |

## From 3D points to a map the planner can use

Nav2 plans on a flat grid. There are several ways to feed it from a 3D LiDAR:

| Way | What it does |
|---|---|
| **Flatten to a 2D scan** | Take the points in a height band and treat them as one flat scan. Simple; what our simulation does |
| **Voxel layer** | Keeps obstacles in a coarse 3D grid and projects them down. Clears old obstacles by tracing rays; that has to be switched on for each sensor [9] |
| **Spatio-Temporal Voxel Layer** | Obstacles fade out after a set time instead of being cleared by rays. Made for 3D sensors and moving things |
| **Height maps** | Store the ground height per cell; used by legged robots on rough ground |

Things to set by hand: the height band (above the floor, below the ceiling), how fast moved obstacles
disappear, and a blind zone around the robot so it does not see its own body.

**Has anything replaced the flat map?** In research, yes: 3D grids, meshes and learned scene models are
used for planning, and one 2026 humanoid paper argues that height maps lose overhangs. In practice, for
navigation of the kind Nav2 does indoors, we found no sign that the flat map has been replaced.

## Options for our robot

1. Keep DLIO for odometry. Compare it once with FAST-LIO2 on a recorded walk if it struggles on the robot.
2. For the position on the stored map, try the global matcher followed by fine tracking; fall back to
   tracking with a start position set by hand.
3. Check on the robot whether Unitree's own mapping service is installed, since it would need no work.
4. Feed the planner through a 3D obstacle layer with fading, so people and moved chairs disappear again.
5. Clean people and one-day clutter out of the stored map with one of the removal programs, and compare
   with our by-hand result.
6. Mark glass walls by hand, in the stored map and in the live data.

## Not found

* A comparison of odometry programs on a humanoid with this LiDAR.
* How fast the matchers run on the robot's onboard computer.
* Tested handling of glass for this LiDAR.

## Sources

1. Direct LiDAR-Inertial Odometry (DLIO), 2022. <https://arxiv.org/abs/2203.03749>
2. SLAM benchmark on Livox Mid-360, forum post, 7 September 2026. <https://discourse.openrobotics.org/t/slam-benchmark-on-livox-mid-360-public-dataset-our-own-uav-flight-data-kiss-icp-dlio-fast-lio2-glim/57912>
3. LIKO: LiDAR, inertial and kinematic odometry for bipedal robots, 2024. <https://arxiv.org/abs/2404.18047>
4. Vibration-aware LiDAR-inertial odometry, 2025. <https://arxiv.org/abs/2507.04311>
5. Unitree, SLAM and Navigation Services Interface. <https://support.unitree.com/home/en/developer/SLAM%20and%20Navigation_service> (content seen only through a copy)
6. KISS-Matcher, 2024. <https://arxiv.org/abs/2409.15615>
7. From blind startup to global relocalization, blog post. <https://dev.to/_pipo/from-blind-startup-to-global-relocalization-building-a-ros-2-3d-lidar-navigation-system-dho>
8. A dynamic points removal benchmark in point cloud maps, 2023. <https://arxiv.org/abs/2307.07260>
9. Nav2 voxel layer. <https://docs.nav2.org/configuration/packages/costmap-plugins/voxel.html>
