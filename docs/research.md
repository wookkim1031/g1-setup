---
layout: default
title: How others do it
---

[Back to all setups](index.html)

# How others make robots walk through buildings

Before building further, we looked at how companies and researchers solve the same problem: a robot that
walks through a building to a goal without hitting furniture or people. Five questions were researched on
17 November 2026, and the key claims were checked against the original pages. This page lists **every
improvement that turned up**, not a selection. The five topic pages give the details and the sources.

| Topic page | Question |
|---|---|
| [Routes and waypoints](research-waypoints.html) | How do commercial robots choose where to walk? |
| [Tight spaces](research-tight-spaces.html) | How do robots get through narrow gaps and avoid getting stuck? |
| [People](research-people.html) | How do robots behave around walking people? |
| [Learned navigation](research-learned.html) | What has machine learning added, and does it replace maps and planners? |
| [Localisation and maps](research-localisation.html) | How does the robot know where it is, and what map does it use? |

## Our starting point

A classical planner (ROS 2 Nav2, release Humble) sends speed commands to a learned walking controller. The
map is a 2D floor plan cut from a LiDAR map. That overall pattern, a planner on top of a learned walking
controller, is what is deployed on commercial walking robots too. The differences are in the details below.

## Every improvement found, in one table

"Effort" is our own estimate for this setup: **small** = a settings change, **medium** = a new component to
set up and test, **large** = a project of its own. "Evidence" says how well the benefit is shown in the
sources: **strong** = official documentation or peer-reviewed results, **medium** = preprints or the
authors' own tests, **weak** = single small tests, blogs or marketing.

### Choosing where to walk

| Improvement | What it does | Who uses it | Effort | Evidence |
|---|---|---|---|---|
| Recorded route graph | The robot follows waypoints and edges recorded once, inside a corridor | Spot (GraphNav), ANYmal | medium | strong |
| Nav2 Route Server | The same idea inside Nav2: a graph of routes replaces or guides free planning | Nav2, since release Kilted (2025) | medium, needs a newer Nav2 | strong |
| Waypoint follower | Visit a list of goals in order, with a task at each | Nav2, in our release | small | strong |
| Keepout and speed zones | Mark areas the robot must not enter, or must cross slowly | Nav2 | small | strong |
| Fleet standard VDA 5050 | One language between robots of different makers and a fleet manager | Warehouse robot makers | not needed for one robot | medium |

### Tight spaces and getting stuck

| Improvement | What it does | Who uses it | Effort | Evidence |
|---|---|---|---|---|
| Real footprint instead of a circle | The planner knows the robot is narrower one way than the other | Nav2 | small | strong |
| Footprint-aware planners | Smac Hybrid-A* and State Lattice plan with the real shape; Smac 2D assumes a circle | Nav2 | small to medium | strong |
| Sideways motion (omnidirectional MPPI) | The controller may command sideways steps | Nav2 | small | strong |
| Wide, smooth safety margin | Routes stay in the middle of aisles | Nav2 tuning advice | small | strong |
| Rotation Shim | Turn towards the path first, then walk | Nav2 | small | strong |
| Custom recovery behaviours | Replace "spin and back up" with moves the robot can do | Nav2 behaviour trees | medium | strong that it is possible; no published recipe for a robot that cannot reverse |
| Learned model of the robot's real motion in the planner | The planner predicts what the robot will really do, not what was commanded | ANYmal, Vision 60 and biped research | large | medium to strong |
| Whole-body planning in clutter | The robot crouches, turns or squeezes through | Research on ANYmal and humanoids (2024 to 2026) | large | medium |
| Newer Nav2 releases | Side-biased safety margin, acceleration limits per direction, delay compensation | Nav2 Jazzy, Kilted, Lyrical | medium (new Ubuntu and ROS release) | strong |

### People

| Improvement | What it does | Who uses it | Effort | Evidence |
|---|---|---|---|---|
| Collision Monitor | Filters the speed command: stop zone, slow zone, minimum time to collision | Nav2, in our release | small | strong |
| Safety-rated laser scanners | Certified sensors with protective fields that grow with speed | Industrial mobile robots | large (hardware) | medium |
| Social costmap layers | Extra cost around detected people, more in front of them | Research add-ons for Nav2 | medium | weak to medium |
| People detection and tracking | Find and follow people in LiDAR or camera data | Research add-ons | medium to large | weak to medium |
| Predicting people's paths | Plan around where a person will be | Research | large | medium |
| Learned social navigation | A trained policy that moves among people | Research | large | weak in real deployments |
| People simulators | Test with simulated walking people (HuNavSim) | Research | medium | strong that it exists; simulation does not replace tests with people |
| Fall-aware safety rules | Stability limits, fall detection, controlled descent | Draft standard ISO 25785-1 | to follow, not to build | weak (draft) |

### Learned navigation

| Improvement | What it does | Who uses it | Effort | Evidence |
|---|---|---|---|---|
| A model picks the goal, a planner drives | Language or vision model chooses the destination; classical code does the path | Mobility VLA (Google) | medium | medium |
| Language to mid-level commands | A model turns an instruction into "forward 75 cm" style steps for a walking policy | NaVILA | large | medium (authors' own tests) |
| Visual navigation foundation models | Navigate from camera images without a map | GNM, ViNT, NoMaD | large | weak: a third-party test found frequent collisions |
| Open-vocabulary maps | A map that knows object names, feeding a normal planner | HOV-SG | large | medium |
| End-to-end humanoid models | One network from camera to joints | Company demos, 2026 preprints | not realistic here | weak |

### Localisation and maps

| Improvement | What it does | Who uses it | Effort | Evidence |
|---|---|---|---|---|
| Global matcher without a start pose | Finds the robot on the stored map from one scan (KISS-Matcher), then fine tracking (small_gicp) | Research, open source | medium | medium |
| Scan-to-map tracking with a manual start pose | Classic approach (NDT, GICP) | Open-source ROS 2 packages | medium | medium |
| Other LiDAR odometry | FAST-LIO2, KISS-ICP, GLIM instead of DLIO | Open source | medium | weak: no clear winner |
| Leg-aware or vibration-aware odometry | Uses the legs' motion or models the shaking | Research | large | medium |
| The robot maker's own mapping service | Unitree documents a mapping and navigation service for its robots | Unitree | small to try | weak: details not readable |
| 3D obstacle layers | Keep height information for obstacles (voxel layer, STVL) | Nav2 | medium | strong |
| Removing moving things from the map | Cleans people and moved furniture out of a stored map | Research tools | medium | medium |
| Handling glass | Detect reflections and remove ghost points | Research | medium to large | weak |

## Not found

Things we looked for and could not find published:

* how commercial fleets (warehouse, hospital, restaurant robots) plan internally;
* an escape strategy for a robot that cannot walk backwards;
* navigation built on the walking controller we use (GEAR-SONIC);
* a fair test where a learned navigation model beats a tuned classical planner on a real humanoid;
* the exact safety distances industrial standards require.
