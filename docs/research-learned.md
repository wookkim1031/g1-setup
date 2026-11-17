---
layout: default
title: Learned navigation
---

[Back to the research overview](research.html)

# Learned navigation: what machine learning has added

Since about 2022, many groups have trained neural networks to navigate: from camera images, from spoken
instructions, sometimes with no map at all. This page lists the main kinds, what they were tested on, and
how they compare with the classical approach of map, planner and controller.

## The kinds of learned navigation

| Kind | Input | Output | Examples |
|---|---|---|---|
| **Visual navigation foundation models** | Camera images and a picture of the goal | Short-range waypoints or speeds | GNM, ViNT, NoMaD |
| **Vision-language navigation** | Camera images and an instruction in words | Mid-level steps or speeds | NaVILA, NavFoM |
| **A language model that only picks the goal** | A video tour of the building and a request | A destination; a classical planner drives | Mobility VLA |
| **Learned local policies** | Sensor data around the robot | Speed commands | X-Mobility, COMPASS |
| **Maps that know object names** | 3D scans with image features | A map a normal planner can use, searchable by words | HOV-SG |
| **End to end for humanoids** | Camera images | All joint commands | Company demos, 2026 preprints |

## What was tested, and how well it did

### NaVILA: instructions for legged robots

A vision-language model reads the camera and an instruction and writes out a small step in words, such as
moving forward by 75 cm. That is turned into a speed command, and a separate learned walking policy
carries it out. The language model runs at about one decision per second on a strong graphics card. In the
authors' own real-world tests, 25 instructions repeated three times on a four-legged robot and a small
humanoid, it succeeded in 88 % [1].

### Visual navigation foundation models, tested by others

A study from March 2026 took five such models (GNM, ViNT, NoMaD and two more) and tried them, without
extra training, on two robots in five places. It reports frequent collisions, confusion between places
that look alike, and worse results with blur or glare [2].

### Mobility VLA: the model picks the goal, classical code drives

A large language model watches a recorded tour of the building and picks the frame that matches a request.
A classical graph built from the tour then produces the path. It found the right goal in 86 % and 90 % of
requests in two settings, and completed 80 % and 85 % of real-world runs from start to finish. When the
model was asked to produce the path itself in simulation, it failed every time [3].

### COMPASS: one policy for many robots

A learned local policy is adapted to each robot type with further training. It outputs speed commands; on
humanoids a learned walking policy carries them out. Its gains are measured against the same group's
earlier learned policy, not against a classical planner [4].

### Learned against classical, on a real robot

One small study put a simple classical method (a potential field) and learned policies on the same
wheeled robot in a hallway. The classical method succeeded on a straight stretch, around a corner and
with a person walking by, and failed in a confined space, where one learned policy succeeded in four of
five tries. The classical method was run only once per situation, so this is an anecdote, not a
measurement [5].

### The BARN challenge

A yearly competition on driving through tightly packed obstacles. Its organisers wrote in 2022 that
navigation in very constrained spaces is far from solved [6]. On the 2026 results page, the top four
scores in simulation lie within 0.01 of each other [7]; we could not read which methods the winners used.

## Where each side wins

| Situation | Better choice, on the evidence found |
|---|---|
| A known building with a stored map | Classical planner |
| Reaching an exact pose | Classical planner |
| Predictable behaviour that can be explained | Classical planner |
| A goal given in words ("go to the kitchen") | A language or vision model, to pick the goal |
| A place never seen before, no map | Learned navigation |
| Very tight spaces | Unsolved for both; some learned results look better |

The two best-supported learned systems above both keep a structured layer underneath: a separate walking
policy in one, a classical graph in the other. Neither goes from camera to motion in one step.

## Not found

* Published navigation work built on the walking controller we use (GEAR-SONIC). Its planner takes speed
  and direction commands, so a navigation layer has to be added, as we did.
* A fair comparison of a learned navigation model with a tuned Nav2 on a real humanoid.

## Options for our robot

1. **Keep the classical planner on the stored map** and let a language model choose the goal from the
   labelled objects on the map. This is the Mobility VLA pattern.
2. **A map that knows object names**, if goals should be found that nobody labelled.
3. **A one-day comparison** of our set-up with one open learned policy on the same goals in simulation:
   goals reached, contacts, time, at least five repeats each.
4. **A language model that gives step-by-step commands**, as NaVILA does, only if instructions should work
   in places without a map.

## Sources

1. NaVILA, 2024. <https://arxiv.org/abs/2412.04453>
2. Can Vision Foundation Models Navigate?, 2026. <https://arxiv.org/abs/2603.25937>
3. Mobility VLA, 2024. <https://arxiv.org/abs/2407.07775>
4. COMPASS, 2025. <https://arxiv.org/abs/2502.16372>
5. Are Learning-Based Approaches Ready for Real-World Indoor Navigation?, 2025. <https://arxiv.org/abs/2507.04086>
6. Autonomous Ground Navigation in Highly Constrained Spaces: Lessons learned from the BARN Challenge, 2022. <https://arxiv.org/abs/2208.10473>
7. The BARN Challenge 2026, results page. <https://people.cs.gmu.edu/~xiao/Research/BARN_Challenge/BARN_Challenge26.html>
