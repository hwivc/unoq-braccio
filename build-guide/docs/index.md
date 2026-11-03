# Rebuild the Braccio cube sorter

You're going to rebuild this project from an empty folder. Each chapter has
**one objective**: why we need it now, what to create, the real code with an
explanation of how it works, a way to check it runs, and what's still missing,
which leads to the next chapter.

## The mind map

Everything grows from one goal: *a robot arm that sorts cubes by itself.* We build
the branches left to right. Each one needs the one before it.

```mermaid
mindmap
  root((Sort cubes with a Braccio))
    Move the arm
      1 Workspace and packages
      2 UNO firmware
      3 Protocol in Python
      4 Serial bridge node
      5 Arm model and launch
    See the table
      6 Camera node
      7 Find cubes
    Learn to pick
      8 kNN pick model
      9 Teaching
    Do the job
      10 Sorting
      11 One launch
      12 Web dashboard
      13 Exhibition
```

And this is where we end up. Every box is a chapter:

![Architecture](img/architecture.png)

[Open the interactive diagram](img/architecture.html){ target=_blank }: click a box to see its source.

## How to use this guide

1. **Two folders.** Keep the original repo for reference, and build yours next to it:

    === "Raspberry Pi / Linux"

        ```bash
        ORIG=~/unoq-braccio            # the original repo
        NEW=~/braccio-rebuild          # yours, built from scratch
        ```

    === "Windows (PowerShell)"

        ```powershell
        $ORIG = "$HOME\Downloads\unoq-braccio"
        $NEW  = "$HOME\braccio-rebuild"
        ```

2. **Read a section, then type its code into your file.** Each code block shows
   the original file and line numbers, so you always know exactly where it goes.
   Typing is the point. The copy button is there for long, boring parts.
3. **Some files are support code:** a chapter will say *"copy this file as-is"*.
   They aren't part of the main idea (a keyboard helper, a settings loader), so
   you read them and move on.
4. **Run the check at the end of every chapter** before moving on.
5. Tick the chapter off:
    - [ ] 1 · [ ] 2 · [ ] 3 · [ ] 4 · [ ] 5 · [ ] 6 · [ ] 7 · [ ] 8 · [ ] 9 · [ ] 10 · [ ] 11 · [ ] 12 · [ ] 13

!!! tip "What runs where"
    Chapters 3, 7, 8 and 13 have checks that run on any PC with Python
    (`pip install numpy pyyaml opencv-python`). Everything else runs on the
    Raspberry Pi with ROS 2 Jazzy and the arm. Set that up first with
    `docs/raspberry-pi.md` in the original repo.

## Serving this guide

From the original repo folder:

```bash
pip install mkdocs-material
python -m mkdocs serve
```

Then open <http://127.0.0.1:8000>. The code blocks are read live from the
repo, so they always match the files.
