# Arduino UNO Q set-up (legacy)

The project now drives the Braccio from a plain **Arduino UNO** over USB
serial (`firmware/braccio_uno_firmware`, `hardware.launch.py`). The web
dashboard runs on the ROS computer: `ros2 run unoq_braccio_driver web_dashboard`
(see the main README).

Everything here only applies to the earlier **Arduino UNO Q** board, which
runs its own Linux and App Lab apps. It is kept for reference and still works
with an UNO Q:

| Folder | What it is |
|---|---|
| `app_lab/` | App Lab apps for the UNO Q: TCP arm agent, camera streamer, combined web agent, smoke test, reusable brick |
| `web_app/` | The old browser dashboard that talked to the UNO Q apps over TCP (no login, no 3D view) |
| `firmware/unoq_braccio_firmware/` | UNO Q MCU sketch |
| `scripts/` | Flash the UNO Q, package App Lab apps and bricks |
| `architecture.md` | UNO Q network layout, ports and apps |

ROS side for the UNO Q: `remote.launch.py` and `tcp_bridge` in `ros2_ws/`
connect to `app_lab/braccio_remote_agent` over the network.
