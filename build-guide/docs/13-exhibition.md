# 13. Exhibition mode

## Objective

The arm keeps the table busy for hours. It makes one recognisable pattern after
another (a line by colour, an arc, colour zones, a swap, a shuffle), rests in
between, and asks for help only when it's really stuck.

## Why we need this now

It's the finished product: a demo that runs itself at a show. It also reuses
everything: `ArmLink` to move and see (chapter 9), the model to pick (chapter 8),
the tricks to celebrate (chapter 5), and the dashboard for settings and the big
screen (chapter 12).

The key idea, from the module's docstring:

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/exhibition.py L1-L22

> Putting a cube **down** on a taught spot is the same motion as picking one up
> there, with the gripper opening instead of closing. So placing needs no new
> teaching, and cubes never land off the table.

## Create

1. `exhibition.py`: replace the copy from chapter 12 with your own. It's pure
   Python with no ROS, so it can be tested anywhere.
2. `exhibition_demo.py`, the node. Add `"exhibition_demo = unoq_braccio_driver.exhibition_demo:main",`

## The code

### Settings

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/exhibition.py L24-L100

The web page's Show tab writes this YAML file; the node re-reads it every round,
so you can change things while it runs.

### Spots: where a cube may go

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/exhibition.py in_window

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/exhibition.py px

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/exhibition.py place_spots

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/exhibition.py cube_size_px

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/exhibition.py free_spots

### Choosing what to pick and where to put it

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/exhibition.py choose_place

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/exhibition.py choose_pick

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/exhibition.py spot_key

There's a bit of randomness, weighted towards far-away and not-recently-used
spots, so the show doesn't look like a loop.

### Patterns

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/exhibition.py L204-L239

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/exhibition.py pattern_targets

A pattern is just **a choice of known spots**, with a colour for each. Each branch
answers "can these spots make this shape with these cubes?", and returns `None` if not.

### Deciding the next move

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/exhibition.py satisfied

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/exhibition.py next_move

This is re-decided from what the camera sees **every time**, so a cube that slips
or a person who moves one doesn't break anything. If a cube is in a target's way,
it's moved aside first.

### The node

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/exhibition_demo.py ExhibitionDemo.run

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/exhibition_demo.py ExhibitionDemo.pattern_round

The loop: next pattern from the playlist → plan targets → move one cube at a time
until `next_move` says done → celebrate → rest → repeat. Type the other
`ExhibitionDemo` methods from the original (`pick`, `place`, `move_cube`, `look`,
`shuffle_round`, `attract`, `rest`, `give_up`…).

## Run it

On any PC, the planner's tests:

```bash
cp $ORIG/ros2_ws/src/unoq_braccio_driver/test/test_exhibition.py $NEW/ros2_ws/src/unoq_braccio_driver/test/
python $NEW/ros2_ws/src/unoq_braccio_driver/test/test_exhibition.py
```

On the arm: the web page → **Show** → set the base window → **Start Exhibition**,
and open `http://<pi>:8000/show` on a TV.

## Done

You've rebuilt the whole system: firmware → protocol → bridge → model → camera →
vision → learning → teaching → sorting → launch → dashboard → exhibition.
Compare your folder with the original (`git diff --no-index $ORIG/ros2_ws $NEW/ros2_ws`).
Where the differences are bugs or simplifications, you've found the next things to learn.
