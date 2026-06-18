# Venus — a robot that explores on its own and maps what it finds

> **MAINTENANCE:** This is a living, plain-language overview of the project written for **readers who
> are not involved in it** — anyone who wants to understand what the robot does without reading the
> code. **Keep it current whenever the robot's behaviour or mission changes**, and update it in the
> **same change** as the code and as its technical sibling [`ALGORITHM.md`](ALGORITHM.md).
> `ALGORITHM.md` is the detailed source of truth (for people working on the project); this file is its
> jargon-free projection. If you edit one, refresh the other so they never disagree, and keep this one
> free of code names, file paths, and protocol detail.

---

## What this is

Venus is a small **autonomous robot** — it drives itself, with no human steering it. You place it in a
walled-off area it has never seen, switch it on, and it sets off on its own: wandering the space,
finding objects, measuring each one, watching out for dangers, and continuously radioing what it
learns back to a laptop, which draws a live map. By the end you have a picture of where everything is,
without anyone ever touching a controller.

## The mission

The robot works inside an arena about **1.5 by 1.5 metres**, ringed by an edge it must not cross.
Its job, entirely on its own, is to:

- **Explore** the whole area.
- **Find an unknown number of "rocks"** (small cube-shaped objects) and, for each one, report its
  **size, colour, and temperature**, and **where it is**.
- **Avoid "mountains"** — large obstacles (think cardboard boxes) it must not bump into.
- **Never fall off a "cliff."** The arena's edges and any interior drop-offs are marked with **black
  tape on the floor**. Driving over the tape means falling — and that ends the mission. So black tape
  is an absolute "do not cross."
- **Send everything home** so a laptop can build the map in real time.

It must do all of this starting from any position, the moment it powers on — no cable, no operator
typing commands during the run.

## What's on the robot

- **Two driven wheels** (plus a small trailing wheel for balance) let it move and turn in place.
- A **downward-looking sensor** that tells floor from black tape — its cliff detector. Crucially it
  sits **ahead of the wheels**, so it spots an edge a little before a wheel would reach it.
- A **forward-looking distance sensor** that notices objects and walls in front.
- An **overhead sensor** that gauges an object's **size** once the robot is right up against it.
- A **colour sensor** and a **temperature sensor** for measuring each rock.

The robot has no screen of its own. It has a small radio link that sends its findings to a laptop
"ground station" running the map software.

## How it explores

The robot repeats a simple, careful cycle:

1. **Look around.** Standing still, it slowly turns to scan a half-circle in front of it, noting the
   direction and distance of any objects it sees.
2. **Visit the nearest object.** It turns to face an object and creeps up to it in stages — fast at
   first, then very slowly for the last few millimetres — until it's perfectly positioned to measure.
3. **Measure it** (size, colour, temperature) and report it.
4. **Back up to where it started** the scan, reversing straight along the path it came in on (backing
   up straight is more accurate than turning around).
5. **Look again** before moving on. By re-spotting objects it already knows, it can correct small
   errors that have crept into its sense of where it is — using those objects as landmarks.
6. **Step into new ground systematically.** If the scan found nothing new, it drives forward a short
   way, and when it reaches the boundary it steps sideways one robot-width and drives back the other
   direction — a lawnmower pattern that guarantees the whole arena is covered.

This loop continues until the arena is fully covered — the robot can no longer step into new
ground in either sideways direction — or the operator stops it. It then **stops on its own**, rather
than driving forward indefinitely as the earlier version did. Throughout, the live map on the ground
station shows the robot's marker advancing lane by lane, and it still sweeps for and classifies rocks
at every position.

## How it measures a rock

Once parked right against a rock, it reads three things: the **overhead sensor** estimates **size**,
the **colour sensor** reads **colour**, and the **temperature sensor** reads how **warm** it is. All
three plus the rock's position are sent to the map.

## How it stays safe

Safety is the robot's top priority and runs constantly in the background, independent of whatever else
it's doing. A dedicated watcher reads the downward sensor non-stop; **the instant it sees black tape,
any forward motion stops immediately.** Because that sensor is mounted ahead of the wheels, the robot
halts with a margin to spare, before a wheel can cross the edge. After stopping it backs away or turns
inward to escape — it never tries to drive *around* an unknown black line, because that line might be
the arena's edge.

It also handles two trickier situations: if a **mountain** blocks the way, it backs off and sidesteps
until the path is clear; and if it gets **boxed into a corner** — pinned between a cliff and an
obstacle — it does a careful scan for the most open direction and commits to a longer move to break
free, giving up gracefully (and stopping) if there's genuinely no way out.

The robot also monitors for cliff/boundary tape during sideways shuffle manoeuvres. If the downward
colour sensor detects the black boundary tape while the robot is strafing sideways around a mountain,
the robot stops immediately and defers to its normal cliff-recovery routine. This closes a previously
known safety gap where a mountain near the arena boundary could cause the robot to strafe off the edge.

## What the operator sees on the ground station

When EXPLORE starts, the ground station immediately shows **"[EXPLORE] mission started"** in the
log. As the robot sweeps each position, detected rocks appear as green dots on the map — every
cycle, not just at the end. When the robot drives to a rock the marker moves forward on the map;
when it returns it retraces the path backward. After each forward hop (when the sweep found
nothing) the marker advances on the map. If the robot gets trapped in a corner, the log shows
**[TRAP] escaping** (amber), then **[TRAP_OK] escaped** (cyan) or **[TRAP_FAIL] mission stopped**
(red) if all attempts fail. If the heading drifts and the re-sweep corrects it, the log shows
**[DRIFT] heading corrected X° from N refs** (amber) — this appears every time a correction is
applied, not only during the RSC test command. The RSC test command now shows the same green
rock-dot display as a normal sweep.

## The bigger picture

Everything the robot senses is sent wirelessly to a laptop, which acts as the **ground station**. The
laptop draws a live map: the robot's path, the rocks it found (with their measurements), the walls and
boundary it traced, and the hazards to avoid. The system is even built to track **two robots at once**
on the same map, each shown in its own colour — a stepping stone toward having robots cooperate, which
is planned future work.

## What's genuinely hard about this

The clever, difficult parts are mostly invisible:

- **Knowing where it is without GPS.** Indoors there's no satellite positioning, so the robot
  estimates its position by counting how far its wheels have turned. Small errors build up over time
  ("drift"), so it periodically re-checks against objects it has already seen and nudges its estimate
  back in line.
- **Telling a wall from an obstacle.** A patch of black tape could be the outer boundary or just a
  small drop-off in the middle of the arena. The robot treats *all* black as dangerous immediately,
  and the map software later works out which black marks form the boundary loop and which are isolated
  interior hazards.
- **Not falling while squeezing past things.** Moving close to edges and obstacles without a wheel
  ever crossing the tape takes careful, slow, well-timed motion.
- **Separating objects that overlap.** The forward sensor sees a fairly wide cone, so two nearby
  objects can blur together; the robot uses the pattern of distances as it scans to tease them apart.

## Current status and honest limits

- It currently runs as a **single robot**; coordinated **two-robot teamwork is planned but not done**.
- Its sense of position **drifts** over long stretches with no landmarks in view; it corrects locally
  but does not build a perfect global map.
- Distinguishing the boundary from interior hazards is a **best-effort guess** made after the run, not
  a guarantee — though the live "don't cross black" safety rule is always in force regardless.
- It can only see **nearby** objects on each scan, so coverage comes from many short hops rather than
  one long-range view.
- There is **one known edge-case safety gap** that the team is tracking.

## Glossary

- **Arena** — the walled-off area the robot explores (about 1.5 × 1.5 m).
- **Cliff** — a drop-off the robot must not cross, marked by black tape (the outer boundary counts as a
  cliff too).
- **Rock** — a small cube the robot finds and measures (size, colour, temperature).
- **Mountain** — a large obstacle (e.g. a box) the robot must avoid hitting.
- **Sweep / scan** — turning in place to look around and locate objects.
- **Drift / dead-reckoning** — estimating position by counting wheel motion; accumulates small errors.
- **Ground station** — the laptop that receives the robot's reports and draws the live map.
- **Autonomous** — acting on its own, with no human steering during the run.

---
*Keep this file current — see the MAINTENANCE note at the top. Its technical counterpart is
[`ALGORITHM.md`](ALGORITHM.md).*
