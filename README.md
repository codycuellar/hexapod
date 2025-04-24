I had a 3d printer. I like technology. I wanted to learn about robots, so here we go, let's build a hexapod!

**_NOTICE: I have no idea what I’m doing, this is all for hobby experimentation and documentation purposes for myself, and is a complete work in progress. I do not condone, recommend or authorize anyone to use any of this code for any purpose, whether explicitly or implied. I do not in any way shape or form attest for its quality, security, or safety. I cannot help if you try to run all 18 servos off your PC USB and fry your computer! Use at your own risk!!_**

My hexapod designs should be added later, but currently my hardware is:

- Pimoroni Servo2040 (micropython firmware)
- Deegoo FPV MG996r servos

# Setup

This assumes you're on mac or using gitbash.

1. Acquire a python 3 version on your system (recommended to use [pyenv](https://github.com/pyenv/pyenv))
2. Clone this repo.
3. CD into the root.
4. Setup a venv: `python -m venv venv`
5. Activate the venv:
   1. mac: `source venv/bin/activate`
   2. win: `source venv/Scripts/activate`
6. `pip install -e .`

# VSCode

Thonny is an option, but I much prefer the dev cycle in VSCode and the extension MicroPico.

## Dev cycle

1. when making changes to core lib files, right click in the project folder window and do 'upload project files to pico'.
2. To execute the code, right click in the 'main.py' file and 'run current file on pico'.
3. To stop, hopefully the reset button on the servo2040 works, or command+c in the integrated terminal, or the stop button in the bottom task bar. If not - keep your power supply off switch handy, or unplug the USB.
