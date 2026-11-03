# 2. Move the servos (UNO firmware)

## Objective

Make the Arduino UNO accept one-line text commands over USB and move all six
servos smoothly, so that they all arrive together.

## Why we need this now

The servos are driven by the Braccio shield on the UNO, so nothing else can move
the arm. We keep the UNO **dumb on purpose**: it only moves servos and reports
where they are. All the thinking (cameras, learning) runs on the Pi, where it's
easier to write, test and change.

The header comment is the specification. Read it first; everything below implements it:

@code firmware/braccio_uno_firmware/braccio_uno_firmware.ino L1-L30

!!! info "Why not the official Braccio library?"
    Its `ServoMovement()` **blocks** until a move finishes, so the UNO can't
    read new commands or report status mid-move. It also caps the gripper at 73°,
    which doesn't close this gripper. So we write our own.

## Create

`$NEW/firmware/braccio_uno_firmware/braccio_uno_firmware.ino`, built up in this order.

## The code

### Constants and state

@code firmware/braccio_uno_firmware/braccio_uno_firmware.ino L32-L64

- **Pins** are fixed by the shield, in joint order: base, shoulder, elbow, wrist vertical, wrist rotation, gripper.
- **Limits** stop a servo from being driven into the arm's own body. They must match `braccio_model.py` (chapter 3).
- **State:** `current` is where each servo is now, and `target` is where it's
  heading. `moveFrom` and `moveStartMs` let us interpolate.
- **`line[48]`** buffers incoming characters until a newline.

### Clamp and soft start

@code firmware/braccio_uno_firmware/braccio_uno_firmware.ino clampJoint

@code firmware/braccio_uno_firmware/braccio_uno_firmware.ino softStart

`softStart` pulses the shield's power pin, so the servos wake up gently instead
of jumping at full power.

### Planning a move

@code firmware/braccio_uno_firmware/braccio_uno_firmware.ino startMove

The joint with the **furthest** to go sets the duration (`degrees × 1000 / speed`
ms). Every other joint is slowed to finish at the same time, so the arm moves
in a straight, calm motion rather than one joint snapping ahead.

### Doing the move, a little every loop

@code firmware/braccio_uno_firmware/braccio_uno_firmware.ino updateMove

This is **linear interpolation**: `position = from + (to − from) × elapsed / duration`.
It runs every `loop()` and returns immediately, so the UNO is never stuck. A new
`M` mid-move just calls `startMove` again from wherever the arm is now.

### Reading and answering commands

@code firmware/braccio_uno_firmware/braccio_uno_firmware.ino parseInts

`strtol` reads one number and moves `end` past it. If `end` didn't move, there was
no number. Trailing junk makes the whole command invalid.

@code firmware/braccio_uno_firmware/braccio_uno_firmware.ino printList

@code firmware/braccio_uno_firmware/braccio_uno_firmware.ino printStatus

@code firmware/braccio_uno_firmware/braccio_uno_firmware.ino handleLine

One letter picks the command. Every command answers, either `OK` or `ERR <reason>`,
so the Pi always knows whether it was understood.

@code firmware/braccio_uno_firmware/braccio_uno_firmware.ino readSerial

Characters are collected until `\n` **or** `\r`. Treating `\r` as an end matters:
Linux's ModemManager sends `AT…\r` probes to new USB devices, and that junk must
not stick to the next real command.

### Start-up and the main loop

@code firmware/braccio_uno_firmware/braccio_uno_firmware.ino setup

@code firmware/braccio_uno_firmware/braccio_uno_firmware.ino loop

The servos are attached and given the start pose *before* power ramps up, so they
wake up already holding it. Then the UNO announces itself with `READY` and a `STAT` line.

## Run it

Copy the flashing script from the original (`cp $ORIG/scripts/flash_uno.sh $NEW/scripts/`), then:

```bash
bash scripts/flash_uno.sh /dev/ttyACM0
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

Wait for `READY BRACCIO_UNO 1`, then type `S`, then `M 90 90 90 90 90 73`, then `S`.
Passes when you see `OK`, then `DONE`, the gripper opens smoothly, and the second `STAT` shows `73`.

## What's next

You can drive the arm by typing, but ROS can't. Next we teach Python to build
these lines and read `STAT` replies: chapter 3.
