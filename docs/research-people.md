---
layout: default
title: People
---

[Back to the research overview](research.html)

# People: how robots behave around walking visitors

A robot in a hall with visitors has to do two things: not walk into anyone, and behave in a way people
find normal. The first is safety engineering with standards behind it. The second is a research field
called social navigation. This page covers both.

## The engineering answer: filter the speed command

Whatever the planner decides, a last check sits between it and the motors. Nav2 ships this as the
**Collision Monitor**, and it is in our release [1]. It watches the sensor data inside shapes drawn around
the robot:

| Zone type | What happens when something is inside |
|---|---|
| **Stop** | The robot stops |
| **Slowdown** | The speed is cut to a set share |
| **Approach** | The speed is reduced so that a set time to collision always remains |

If several zones react, the strictest wins. The third type scales with the robot's own speed, which fixed
distances do not. Newer Nav2 releases let the zones change shape with speed (Jazzy, 2024) [2] and add
zones that are ignored on purpose (Lyrical, 2026) [3].

Industrial mobile robots do the same with **certified safety laser scanners**, whose protective fields
grow with speed. The stopping distance is worked out from how fast a person may approach and how long the
robot needs to stop. We did not manage to read the numbers the industrial standards require.

## Safety standards for walking robots

| Standard | What it covers | Status |
|---|---|---|
| ISO 13482 | Personal care robots, including mobile servant robots: safe stopping, speed limits that depend on the task | Published 2014; numbers not read |
| ISO 3691-4, ANSI R15.08 | Industrial mobile robots | Published; they do not cover robots that balance |
| **ISO 25785-1** | Industrial mobile robots that balance actively: legged and two-legged robots | **A draft.** Its comment period closed in July 2026; the current stage is not confirmed [4] |

A commentary on an earlier draft of ISO 25785-1 expects it to ask for limits on stability, detecting a
fall, a controlled way of going down, and lower speed near people [5]. That is one author's reading of a
draft, not the final text. The point for a humanoid is that **the robot can fall**, and a falling 35 kg
robot is itself the hazard. Stopping is only safe if the robot stays standing when it stops.

## Social navigation research

### Method families

| Family | Idea | Known weakness |
|---|---|---|
| Reactive (social forces, ORCA) | People and robot push each other away like magnets | Ignores how the robot can actually move, which matters for a slow robot that cannot reverse |
| Prediction plus planning | Predict each person's path, then plan around it | If prediction and planning are separate, the robot can decide every path is unsafe and freeze |
| Learned policies (reinforcement learning) | Train a policy among simulated people | Works in simulation; moving to real robots stays hard [6] |
| Learning from demonstrations | Copy how people drove a robot through crowds | Depends on the demonstrations that were collected |
| Hybrid | A classical planner most of the time, a learned one where it helps | Most promising in the tests we found |

### What the evidence says

* A 2024 study compared classical planners with human demonstrations of driving through public places.
  The classical planners matched what the humans did in most situations, up to 80 % of them. A hybrid that
  switched between a classical and a learned planner scored best in most, but not all, of the tests on two
  real robots [7].
* A late-2025 survey of learned social navigation reports that moving from simulation to real robots is
  still a problem, and that the field has no agreed way of measuring success [6].
* We found **no real deployment** in which a learned social policy clearly beats a well-tuned classical
  one. That is "not found", not "proven impossible".

### How to measure it

A large group of researchers published guidelines for evaluating social navigation [8]. They recommend
reporting three kinds of results:

* the usual ones: goals reached, time, path length, collisions;
* social ones: how close the robot came to people and for how long;
* what people say in questionnaires.

They also state that simulation is useful for finding problems but does not replace tests with real
people. As an illustration they use the distances from the study of personal space: about 0.45 m as
intimate and 1.2 m as personal. These are illustrations, not required thresholds.

### Tools

* **HuNavSim**: an open simulator of walking people for ROS 2, with several behaviours (for example people
  who ignore the robot, or who are curious about it) and built-in measurements [9].
* **Social costmap layers** and **people trackers** for Nav2 exist as research add-ons, for example with a
  3D LiDAR and a depth camera together. We did not check whether they are maintained.

## What we have so far

Our simulator now has walking people (cylinders at 1.0 to 1.4 m/s that cross the robot's path) and a
hand-written stop and slow zone. Our people still crowd the robot more than real visitors would, so the
first numbers need care.

## Options for our robot

1. Replace the hand-written zone with Nav2's Collision Monitor, including the time-to-collision zone.
2. Work out the stop distance from measurements: how long the robot needs to come to a stand from a stop
   command, and how fast a visitor approaches.
3. Report the measures from the guidelines: closest distance, time closer than 0.45 m and 1.2 m, stops,
   and how long the robot stays blocked.
4. Use HuNavSim's people models instead of our own simple ones.
5. Add a social cost layer around detected people only if the simpler set-up still comes too close.
6. On the real robot: slow walking, a person with an emergency stop, and a check that stopping leaves the
   robot standing.

## Not found

* The distances and speeds industrial standards require.
* How commercial service robots behave when a person blocks them.
* Any standard specifically for humanoids in public spaces.

## Sources

1. Nav2 Collision Monitor, README for Humble. <https://api.nav2.org/nav2-humble/html/md_nav2_collision_monitor_README.html>
2. Nav2 Jazzy Release, 24 June 2024. <https://discourse.openrobotics.org/t/nav2-jazzy-release/38321>
3. Nav2 Lyrical Release and Key Features, 25 August 2026. <https://discourse.openrobotics.org/t/nav2-lyrical-release-key-features/57666>
4. ISO/CD 25785-1. <https://www.iso.org/standard/91469.html> (status seen through a national standards body's listing)
5. i-Scoop, ISO 25785-1 explained. <https://www.i-scoop.eu/iso-25785-1-explained-and-what-it-means-for-humanoid-robot-safety/> (commentary)
6. Socially aware navigation: a survey on deep reinforcement learning approaches, 2025. <https://arxiv.org/abs/2512.00049>
7. Raj et al., Rethinking Social Robot Navigation, 2023 (ICRA 2024). <https://arxiv.org/abs/2309.13466>
8. Francis et al., Principles and Guidelines for Evaluating Social Robot Navigation, 2023. <https://arxiv.org/abs/2306.16740>
9. HuNavSim, 2023. <https://arxiv.org/abs/2305.01303>
