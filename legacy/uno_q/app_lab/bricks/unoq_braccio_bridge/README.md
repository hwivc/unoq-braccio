# UNOQ Braccio Bridge Brick

Reusable App Lab brick for controlling a TinkerKit Braccio shield from an
Arduino UNO Q.

This brick provides:

- Standard Braccio shield servo pin mapping.
- Conservative joint limits.
- Smooth stepped servo movement.
- A `move_braccio(...)` function compatible with `Arduino_RouterBridge`.

## Files

```text
brick.yaml
sketch/UnoQBraccioBridge.h
sketch/UnoQBraccioBridge.cpp
examples/basic_bridge/
```

## API

```cpp
void setupBraccioBridge();

bool move_braccio(
  int base_angle,
  int shoulder_angle,
  int elbow_angle,
  int wrist_vertical_angle,
  int wrist_rotation_angle,
  int gripper_angle
);
```

## Servo Pins

| Joint | Pin |
| --- | ---: |
| base | 11 |
| shoulder | 10 |
| elbow | 9 |
| wrist vertical | 6 |
| wrist rotation | 5 |
| gripper | 3 |
| soft start | 12 |

The gripper range is `10-110` for this build. Start with `95` before using
`110`.
