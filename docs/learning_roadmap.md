# Learned picking: what next

Where the taught (kNN) real-arm picking stands, what to clean up, and a plan
for colours, stacking, building and reordering, with the model each step
uses.

## Where it stands

- `teach_pick` records examples: camera reading (cube centre `u, v`, angle)
  and the servo angles of an *above* and a *grab* pose.
- `pick_learning` maps a camera reading to those servo angles. Every 5
  examples it scores kNN, polynomials (degree 1-3) and a small neural network
  (MLP) on examples they did not see and keeps the best. With ~47 hand-taught
  examples kNN wins (~1.6-1.8 degrees error, the others ~3).
- Wrist roll is computed: cube angle plus a learned offset.
- `learned_pick_demo` picks every cube it knows, or only some colours
  (`colors:=red`), checks with the camera, and drops it at a taught pose or at
  the far end of the base rotation (`drop:=side`).

What it cannot do: reach outside the taught area, pick or place above table
level, or place anywhere other than the drop poses.

## Now: clean the examples (session `new`)

The review (`R` in `teach_pick`) found these. Deleting or re-teaching them
should do more for the success rate than any model change.

| Example | Problem | What to do |
|---|---|---|
| #43 at (283, 135) px | grab shoulder 126 where neighbours use ~57 | delete |
| #45, #46, #47 vs #14, #15 (top left, ~200-285, 50-65 px) | two postures for one area: base 26-36 vs 56-57 | keep one posture, delete the other set, re-teach the area consistently |
| #6, #11, #12, #16, #37 | wrist 10-16 degrees off its neighbours | check; delete if the posture was odd |
| #13, #27, #44 | camera wobble 6-9 px when recorded | delete and re-record with the cube still |
| miss at (276, 172) px | nothing taught near it since | teach 2-3 examples there |

Then run 10 tests spread over the table; aim for 9/10 before adding features.

## Next: Gaussian process as a model candidate

A Gaussian process (GP) is the model most likely to beat kNN at 50-100
examples:

- Like kNN it leans on nearby examples, but it learns from the data how far
  each example's influence should reach, and gives smooth in-between poses
  instead of a weighted average.
- It reports how sure it is at every spot, which can replace the fixed 60 px
  "too far" rule with "uncertainty too high".
- It needs only numpy, so it is one more class in `pick_learning.candidates()`.
  The existing cross-validation decides whether it really wins; nothing else
  changes.

Also worth trying as a candidate: a **local linear fit** (a small plane
through the neighbours instead of their average), which is a little better
between and just beyond examples.

No model fixes examples that disagree: they all average their targets, so
clean data comes first.

## Later: more colours

Works today. Colour is never a model input, so a new colour needs no new
examples:

1. `ros2 run unoq_braccio_driver real_setup --ros-args -p steps:=colors`
   (existing colours are kept), restart `real.launch.py`.
2. Optional drop pose: `D` in `teach_pick`.
3. `learned_pick_place.launch.py session:=new colors:=<colour>`.

Risk: two colours with close hues (orange/red, cyan/blue). The colours step
names them by hue, so check the camera tells them apart under your lighting.

## Later: stacking, building, taking apart, reordering

kNN maps "cube flat on the table at this pixel" to servo angles. Stacking
needs things it has never seen: other heights, placing anywhere, and ±3 mm
precision. Teaching every height and every place spot by hand does not scale,
so the plan splits the job into three layers:

```text
camera ──> 1 perception ──> 2 planner ──> 3 reach ──> servos
          where cubes are,  which cube     move the gripper
          what is on what   goes where     to (x, y, height, angle)
```

| Layer | Job | Model / method |
|---|---|---|
| 1 Perception | Cube positions (x, y on the table), colour, angle; stack height | Colour detection (today). Height: track what the arm stacked; check with the cube's apparent size (a cube one level up looks ~5-10 % bigger from above). Edge Impulse model if colours get unreliable. |
| 2 Planner | Turn a goal ("red on blue on green", "undo the tower", "sort by colour") into pick/place steps; order them so nothing is buried | Plain logic, no learning: a list of target slots, picked top-down, placed bottom-up. Re-plan after every move from what the camera sees. |
| 3 Reach | Any (x, y, height, angle) to servo angles, accurately | Arm geometry (inverse kinematics, already in `braccio_kinematics`) **plus a learned correction**: GP or kNN trained on the taught examples' *error* of the geometry. The geometry reaches anywhere and any height; the correction removes this arm's servo offsets and bends. |

Why geometry plus correction: your taught examples stay useful (they become
the correction's training data), it works at heights and spots never taught,
and the posture always comes from the same rule, so the "two postures
averaged" problem disappears.

### Steps, in order

1. **Pixel to table position.** Learn camera pixel to table (x, y) in mm from
   the taught examples (forward kinematics of each grab pose gives where the
   gripper actually was). Model: polynomial or GP; it is a smooth map.
2. **Reach with correction.** IK for (x, y, z), plus a GP on the residual
   (taught servo angles minus IK angles). Test: pick at table level should be
   at least as good as kNN today. Keep kNN as a fallback.
3. **Place anywhere.** Use the same reach to place a held cube at a given
   (x, y). Test: line up 3 cubes 4 cm apart.
4. **Stack 2, then 3.** Place at z = level x cube size, slow final descent,
   open, lift straight up. After each place, the camera checks the top cube is
   centred on the one below (within ~3 mm); if not, pick it again and retry.
   Teach a handful of examples at level 1 and 2 so the correction learns the
   arm's sag when reaching high.
5. **Build shapes.** A shape is a list of slots (x, y, level, colour), e.g.
   tower, row, pyramid of 3, wall of 2x2. The planner fills slots bottom-up.
6. **Take apart.** Pick top-down to scattered free spots on the table.
7. **Reorder.** Goal state vs current state; the planner moves cubes that are
   out of place, using a free spot as a buffer when a cube is in the way
   (like Tower of Hanoi).

### Data to collect along the way

- Every test and every pick attempt: reading, predicted pose, result (already
  logged in `tests.jsonl`). This is the training set for the correction and
  shows where the arm is weak.
- A few examples per stack level, once step 4 starts.

## Summary: which model where

| Task | Model |
|---|---|
| Pick from table, taught area (today) | kNN, chosen automatically over poly / MLP |
| Pick from table, 50-100 examples | Gaussian process candidate (likely winner) |
| Pick and place anywhere, any height | Inverse kinematics + GP correction |
| Which cube, which order | Planner (rules, no learning) |
| Colours | Colour detection; Edge Impulse model if needed |
| Later, with hundreds of picks logged | MLP correction, or closed-loop camera nudging before the grab |
