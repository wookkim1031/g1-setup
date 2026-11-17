---
layout: default
title: Tight spaces
---

[Back to the research overview](research.html)

# Tight spaces: narrow gaps and getting stuck

A walking robot is not a circle, does not move exactly as commanded, and often cannot step backwards. All
three matter in narrow places. This page lists what the navigation software offers for that, what
researchers have added, and what nobody has published.

## What we saw ourselves

In simulation, with the robot described to the planner as a circle 0.64 m wide, it could not pass gaps a
person would walk through. Once its centre came closer to an obstacle than its radius, every plan, turn and
back-up was refused and it stood still. Described by its real shape, about 0.34 m deep and 0.48 m wide,
and allowed to step sideways, it passed the same places. Its hands then brushed obstacles a few times,
because they stick out past that width.

## What Nav2 offers

| Feature | What it does | Note |
|---|---|---|
| **Footprint instead of radius** | The robot is described by its outline, not a circle | Available in our release |
| **Footprint checking in the MPPI controller** | The controller tests the whole outline against obstacles. It is **switched off by default**, in which case only the centre point is tested [1] | Easy to miss |
| **Omnidirectional model in MPPI** | The controller may command sideways motion; a setting gives the highest sideways speed [1] | Available in our release |
| **Controller time step** | MPPI's time step has to match how often the controller runs, or it will not start [1] | We hit this |
| **Smac Hybrid-A\* and State Lattice planners** | Plan with the real outline and with how the robot can actually move. The planner we use, Smac 2D, is meant for round robots. The Hybrid planner lists walking robots as supported, and the Lattice planner comes with motion sets for them [2] | A stronger fix than our current one |
| **Smooth, wide safety margin** | The planner documentation advises spreading the cost far from obstacles, so paths run down the middle of aisles [2] | We did this |
| **Rotation Shim** | Turns the robot towards a new path before it starts walking. Useful with simple planners, unnecessary with the footprint-aware ones [3] | Needs a robot that turns on the spot |
| **Recovery behaviours** | By default: clear the obstacle map, turn about 90 degrees, wait five seconds, back up 30 cm, up to six tries [4]. The list is a file that can be replaced by your own | Two of the four do not suit a robot that cannot reverse |

Newer Nav2 releases add more. Jazzy (2024) made MPPI about 45 % faster and added acceleration limits [5].
Lyrical (August 2026) added, for MPPI, a model of the robot's dynamics, different acceleration limits per
direction and compensation for delay, plus a safety margin that prefers passing on one side [6]. Different
limits per direction is what a robot that walks forwards well and backwards not at all needs.

## The robot does not move as commanded

Our robot reaches about three quarters of the commanded forward speed and lags in turns. The planner
assumes the command is carried out exactly, so its predictions are off. Research groups handle this by
teaching the planner how the robot really responds:

| Work | Robot | What they did | Reported result |
|---|---|---|---|
| Learned forward dynamics in MPPI (RSS 2025) [7] | ANYmal (four legs) | A learned model predicts where the robot will really be | Position predicted 41 % better; 27 % more goals reached in rough simulated terrain |
| Neural observer-predictor in MPPI (2025) [8] | Vision 60 (four legs) | A learned model predicts the whole body, so collisions are checked per limb | Passed a narrow passage; no numbers in the summary |
| Koopman model with predictive control (2024) [9] | A two-legged robot | A simple learned model of the walking policy's response to speed commands | More successes in narrow passages |

No source gives a simple rule such as "add this much margin for a robot that reaches 80 % of the command".

## Robots that squeeze through

Some research goes further and lets the robot change its posture:

* **Confined spaces with a four-legged robot (ICRA 2024)** [10]: a two-level learned controller that uses a
  3D picture of the surroundings to pass under and between obstacles.
* **PASSAGE (September 2026)** [11]: a planner paired with a whole-body controller that sees its
  surroundings, on a Unitree G1. In simulation on unseen layouts, 98.7 % of runs reached the goal and
  70.3 % did so without touching anything. On the real robot, 50 of 50 runs reached the goal and 45 were
  free of contact. These are the authors' own results.
* **HumanoidPF and FocusNav (January 2026)** [12]: humanoid navigation through narrow passages; FocusNav
  runs on a G1.

These are research systems trained on large amounts of motion data, not settings one can switch on.

## Someone did the same thing we did

A public project, `rl_hnav` from a university robotics group, runs Nav2 with a learned walking policy for
the Unitree G1 in the MuJoCo simulator [13]. A small bridge program limits and smooths the speed command
and stops the robot when commands stop arriving. The project reports no test results.

## Not found

* A published way to free a robot that is stuck and cannot walk backwards.
* What Nav2's planners do when the robot already stands inside an obstacle's margin; the documentation
  does not say for our release.

## Options for our robot

1. Widen the footprint by a few centimetres to cover the hands, or hold the arms closer.
2. Try the Hybrid-A\* or Lattice planner, which plan with the real shape from the start.
3. Write an escape behaviour that fits the robot: turn towards the open side, step sideways, walk out.
4. Measure how the robot really responds to speed commands and tell the controller, first as simple limits,
   later perhaps as a learned model.
5. Move to a newer Nav2 for per-direction limits and delay compensation.

## Sources

1. Nav2 MPPI controller, README for Humble. <https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/README.md>
2. Nav2 Smac planner, README for Humble. <https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/README.md>
3. Nav2 Rotation Shim controller, README for Humble. <https://github.com/ros-navigation/navigation2/blob/humble/nav2_rotation_shim_controller/README.md>
4. Nav2 default behaviour tree with recovery, Humble. <https://github.com/ros-navigation/navigation2/blob/humble/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml>
5. Nav2 Jazzy Release, 24 June 2024. <https://discourse.openrobotics.org/t/nav2-jazzy-release/38321>
6. Nav2 Lyrical Release and Key Features, 25 August 2026. <https://discourse.openrobotics.org/t/nav2-lyrical-release-key-features/57666>
7. Learned perceptive forward dynamics with MPPI, 2025. <https://arxiv.org/abs/2504.19322>
8. Neural observer-predictor models with MPPI, 2025. <https://arxiv.org/abs/2510.22789>
9. Koopman-based safe navigation for bipedal robots, 2024. <https://arxiv.org/abs/2409.14736>
10. Miki et al., Learning to walk in confined spaces using 3D representation, 2024. <https://arxiv.org/abs/2403.00187>
11. PASSAGE, 2026. <https://arxiv.org/abs/2609.18732>
12. HumanoidPF, 2026. <https://arxiv.org/abs/2601.16035>; FocusNav, 2026. <https://arxiv.org/abs/2601.12790>
13. rl_hnav. <https://github.com/uleroboticsgroup/rl_hnav>
