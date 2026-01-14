I had a 3d printer. I like technology. I wanted to learn about robots, so here we go, let's build a hexapod!

**_NOTICE: I have no idea what I’m doing, this is all for hobby experimentation and documentation purposes for myself, and is a complete work in progress. I do not condone, recommend or authorize anyone to use any of this code for any purpose, whether explicitly or implied. I do not in any way shape or form attest for its quality, security, or safety. I cannot help if you try to run all 18 servos off your PC USB, or plug in a power supply and forget to sever the usb backpowering strip on the servo2040 and fry your computer! Use at your own risk!!_**

My hexapod designs should be added later, but currently my hardware is:

- Pimoroni Servo2040 (micropython firmware)
- Deegoo FPV MG996r servos

# Setup (monorepo quickstart)

This repo has three deployments:
- `controller/` (MicroPython on Pico) — Deploys to Servo2040 board and runs in MicroPython.
- `hexapod/` (Python on Raspberry Pi) - Deploys to Raspberry Pi to run 3d engine/kinematics send/receive data to/from servo2040 controller.
- `sim/` (dev machine) - Contains visualization programs to help with gait control, input filtering, etc.

## Create a venv
1. Install Python 3.10+ (e.g., via [pyenv](https://github.com/pyenv/pyenv)).
2. Clone the repo and `cd` into it.
3. Create a venv: `python -m venv venv`
4. Activate:
   - mac/linux: `source venv/bin/activate`
   - windows: `venv\\Scripts\\activate`

## Install extras (pick what you need)
- Shared package only: `pip install -e .`
- Hexapod (RPi service dev): `pip install -e .[hexapod]`
- Sim (dev machine): `pip install -e .[sim]`
- All desktop deps: `pip install -e .[hexapod,sim]`

Controller (Pico) uses MicroPico to sync files; no pip deps are needed on the board.

# VSCode

Thonny is an option, but I much prefer the dev cycle in VSCode and the extension MicroPico. If you use the VSCode workspace, and install the required extensions, VSCode can automatically connect to the servo2040 and give you a REPL and upload the project easily.

