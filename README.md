I had a 3d printer. I like technology. I wanted to learn about robots, so here we go, let's build a hexapod!

**_NOTICE: I have no idea what I’m doing, this is all for hobby experimentation and documentation purposes for myself, and is a complete work in progress. I do not condone, recommend or authorize anyone to use any of this code for any purpose, whether explicitly or implied. I do not in any way shape or form attest for its quality, security, or safety. I cannot help if you try to run all 18 servos off your PC USB, or plug in a power supply and forget to sever the usb backpowering strip on the servo2040 and fry your computer! Use at your own risk!!_**

The current hardware I'm using:
- Pimoroni Servo2040 (micropython firmware)
- Deegoo FPV MG996r servos
- Raspberry Pi 3 B+
- Dual Shock Controller

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

# Raspberry Pi Deployment

## One-Time Setup

For a fresh Raspberry Pi (works on GUI or headless Raspbian):

1. **Connect Raspberry Pi to network** (WiFi or Ethernet)
2. **Clone the repository:**
   ```bash
   git clone <your-repo-url> ~/hexapod
   cd ~/hexapod
   ```
3. **Run the setup script:**
   ```bash
   chmod +x scripts/setup-rpi.sh
   ./scripts/setup-rpi.sh
   ```

The setup script will:
- Install system dependencies (Python, pip, git)
- Create a virtual environment
- Install the hexapod package with dependencies
- Add your user to the `dialout` group (for serial port access)
- Enable user service lingering (allows auto-start without login)
- Install and enable the systemd service
- Start the service

**After setup:** The service will automatically start on every boot, even without logging in.

## Daily Operation

Once setup is complete:
- **Power on Raspberry Pi** → Service automatically starts
- **Connect controller via Bluetooth/USB** → Service connects automatically
- No manual intervention needed

## Service Management

Useful commands for managing the service:

```bash
# Check service status
systemctl --user status hexapod

# View live logs
journalctl --user -u hexapod -f

# Start/stop/restart service
systemctl --user start hexapod
systemctl --user stop hexapod
systemctl --user restart hexapod

# Check if service is enabled (will start on boot)
systemctl --user is-enabled hexapod
```

## Updating the Code

For ongoing development and updates:

1. **Via deployment script (recommended):**
   ```bash
   cd ~/hexapod
   ./scripts/deploy-rpi.sh
   ```
   This script will:
   - Pull latest changes from git
   - Update Python dependencies
   - Update systemd service if changed
   - Restart the service

2. **Manual update:**
   ```bash
   cd ~/hexapod
   git pull
   source venv/bin/activate
   pip install -e .[hexapod]
   systemctl --user restart hexapod
   ```

## Troubleshooting

**Service won't start:**
- Check logs: `journalctl --user -u hexapod -n 50`
- Verify virtual environment exists: `ls ~/hexapod/venv/bin/hexapod`
- Check user is in dialout group: `groups | grep dialout`
- If not in dialout group: `sudo usermod -aG dialout $USER` (then log out/in)

**Serial port not found:**
- Check Servo2040 is connected: `ls -l /dev/ttyACM*`
- Verify permissions: `ls -l /dev/ttyACM0` (should be readable by dialout group)

**Service doesn't start on boot:**
- Verify lingering is enabled: `loginctl show-user $USER | grep Linger`
- Enable lingering: `loginctl enable-linger $USER`
- Check service is enabled: `systemctl --user is-enabled hexapod`

**Bluetooth controller not connecting:**
- Pair controller first: `bluetoothctl`
- Service will attempt to connect automatically on startup

