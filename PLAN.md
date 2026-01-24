# Hexapod Repository Standardization and Deployment Plan

## Current State Analysis

Your current structure is **already following Python best practices**:

- ✅ `/src` layout is a widely-used best practice for Python projects (orthogonal to PEP 420)
- ✅ `pyproject.toml` with setuptools and optional extras is well-configured
- ✅ VSCode workspace with separate folder contexts is clean
- ✅ Separation of concerns: `controller/` (MicroPython), `hexapod/` (RPi), `sim/` (dev)

**Gaps identified:**

1. No main entry point script for hexapod application (Raspberry Pi service)
2. No systemd service file for bootup automation on Raspberry Pi
3. No deployment scripts or documentation
4. `src/hexapod.egg-info/` should be cleaned (gitignored but present)

## Structure Recommendations

**Keep the `/src` layout** - this is a widely-used best practice for Python projects (separate from PEP 420). Your `pyproject.toml` already configures it correctly with `[tool.setuptools.packages.find] where = ["src"]`.

**Note on PEP 420:** [PEP 420](https://peps.python.org/pep-0420/) defines *implicit namespace packages* - packages that can be split across multiple directories and do **not** require `__init__.py` files. Your `hexapod` package uses a **regular package** structure (with `__init__.py`), which is the standard approach and fully PEP 420-compliant. According to PEP 420: "Regular packages will continue to have an `__init__.py` and will reside in a single directory." The `/src` layout is a separate organizational choice (not covered by PEP 420) that's compatible with both regular packages and namespace packages. Your current structure follows Python best practices.

Proposed additions:

- `src/hexapod/main.py` - Entry point for RPi service
- `scripts/` - Deployment and utility scripts
- `deploy/` - Deployment configuration files (systemd services, etc.)
- `.vscode/settings.json` - Workspace-level settings (optional)

## Implementation Steps

### 1. Create Main Entry Point for Hexapod Application

Create `src/hexapod/main.py` that:

- Initializes the hexapod body and motion planner
- Connects to gamepad (USB/Bluetooth)
- Opens serial connection to Servo2040
- Runs main control loop
- Handles graceful shutdown
- Supports both dev mode (direct run) and service mode (systemd)

This will serve as the executable script for Raspberry Pi deployment.

### 2. Add Entry Point Configuration to pyproject.toml

Add console script entry point:

```python
[project.scripts]
hexapod = "hexapod.main:main"
```

This allows installation as `hexapod` command after `pip install -e .[hexapod]`.

### 3. Create Systemd Service for Raspberry Pi

**Installation Strategy:**

For development and production, use a **virtual environment** with an **editable install**:

1. **Clone repository on Raspberry Pi:**
   ```bash
   # Recommended location for user service
   cd ~/hexapod  # or /home/pi/hexapod
   git clone <your-repo-url> .
   ```

2. **Create and activate virtual environment:**
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

3. **Install package in editable mode with hexapod extras:**
   ```bash
   pip install -e .[hexapod]
   ```

   **Why editable (`-e`)?**
   - Development: Changes are immediately available without reinstalling
   - Production: Still works well; allows easy updates via `git pull` + restart
   - Alternative: Use regular `pip install .[hexapod]` for production if you prefer

4. **Create systemd user service file:**
   - Location: `~/.config/systemd/user/hexapod.service`
   - Service activates venv and runs the `hexapod` command
   - Handles USB device availability (waits for Servo2040)
   - Sets up proper permissions for serial ports

**Service Configuration (`deploy/hexapod.service`):**

- Runs as **user service** (no root required, easier permissions)
- **Auto-starts on boot** (with systemd user service lingering enabled)
- Auto-restarts on failure
- Logs to journald
- Waits for USB device availability (Servo2040 via `dev-ttyACM0.device`)
- Sets up environment (activates venv, ensures Python path)
- Handles serial port permissions via `dialout` group

**One-Time Setup (for replication):**

After initial setup, the service auto-starts on every boot:

1. **Clone and install (one-time):**
   ```bash
   cd ~/hexapod
   git clone <repo-url> .
   python3 -m venv venv
   source venv/bin/activate
   pip install -e .[hexapod]
   ```

2. **Enable user service lingering (one-time):**
   ```bash
   # Allows user services to start without login (works on GUI or headless)
   loginctl enable-linger $USER
   ```

3. **Install and enable service (one-time):**
   ```bash
   mkdir -p ~/.config/systemd/user
   cp deploy/hexapod.service ~/.config/systemd/user/
   systemctl --user daemon-reload
   systemctl --user enable hexapod.service
   systemctl --user start hexapod.service
   ```

**Daily Operation (after setup):**

- **Power on Raspberry Pi** → Service automatically starts (even without GUI login)
- No manual intervention needed
- Service runs in background, connects to controller automatically
- To check status: `systemctl --user status hexapod`
- To view logs: `journalctl --user -u hexapod -f`

**GUI vs Headless Raspbian:**

- **GUI Raspbian:** Service still auto-starts on boot (if lingering enabled)
- **Headless:** Works identically
- User services work the same in both cases - GUI login is not required

**For Someone Else to Replicate:**

They follow the same one-time setup process (git clone, install deps, enable service), then it auto-starts on every boot. The repository can include a setup script to automate this.

**Alternative: System-wide installation (not recommended):**
If you prefer a system-wide install instead of venv:
```bash
sudo pip3 install -e .[hexapod]
# But this can cause dependency conflicts and is harder to manage
```

### 4. Add Deployment Scripts

Create `scripts/setup-rpi.sh` (one-time initial setup):
- Installs git, Python, pip if needed
- Clones repository (or syncs if already exists)
- Creates virtual environment
- Installs package with hexapod extras
- Adds user to `dialout` group (for serial port access)
- Enables user service lingering (allows auto-start without login)
- Installs and enables systemd service
- Starts service

Create `scripts/deploy-rpi.sh` (for updates after initial setup):
- Syncs git repository to Raspberry Pi via SSH
- Installs/updates Python dependencies (if pyproject.toml changed)
- Reloads and restarts service
- Useful for ongoing development/debugging

**One-Time Setup Workflow:**

For a fresh Raspberry Pi (GUI or headless):

1. **Connect to network** (WiFi/Ethernet)
2. **Run setup script:**
   ```bash
   curl -fsSL https://your-repo/setup-rpi.sh | bash
   # OR if repo already cloned:
   cd ~/hexapod && ./scripts/setup-rpi.sh
   ```

3. **After setup completes:**
   - Service is enabled and running
   - On every boot: Service auto-starts (no login required)
   - Works on GUI Raspbian: Service starts even before GUI login
   - Works headless: Service starts immediately on boot

**Daily Operation (after setup):**
- Power on Raspberry Pi → Service starts automatically
- Connect controller via Bluetooth → Service connects automatically
- No manual intervention needed

**For Updates/Debugging:**
- Use `scripts/deploy-rpi.sh` to sync changes and restart
- Or manually: `git pull && systemctl --user restart hexapod`


### 5. Add Development Workflow Documentation

Update `README.md` with:

- Raspberry Pi deployment instructions
- SSH development workflow
- Service management commands
- Troubleshooting for serial/bluetooth connections

## Deployment Strategy Recommendation

**Recommendation: Git sync + SSH development workflow** (not containers)

**Rationale:**

- Raspberry Pi with GUI is slow - SSH remote development is faster
- Direct hardware access (USB serial, Bluetooth) is simpler without containerization
- Systemd service is lightweight and integrates with Linux boot
- Container would add complexity for USB device passthrough
- Easier to debug in native environment

**Workflow:**

1. **Dev on Windows:** Write code, test simulations locally
2. **Deploy to RPi:** Use VSCode Remote SSH extension
3. **On RPi:** Git checkout/pull, install deps, run service
4. **Production:** Service auto-starts on boot

## File Structure After Changes

```
hexapod/
├── src/
│   ├── controller/          # MicroPython (Servo2040)
│   │   └── main.py
│   ├── hexapod/             # Python package (RPi + dev)
│   │   ├── __init__.py
│   │   ├── main.py          # NEW: Entry point
│   │   ├── engine.py
│   │   ├── gamepad.py
│   │   ├── motion_planner.py
│   │   ├── rigid_body.py
│   │   ├── servos.py
│   │   └── ...
│   └── sim/                 # Dev-only simulations
│       └── ...
├── deploy/                   # NEW: Deployment configs
│   └── hexapod.service      # NEW: Systemd service
├── scripts/                  # NEW: Utility scripts
│   └── deploy-rpi.sh        # NEW: Deployment helper
├── pyproject.toml           # UPDATE: Add entry point
├── README.md                # UPDATE: Add deployment docs
└── .gitignore
```

## Environment Considerations

The hexapod main code should detect environment:

- **Dev mode:** Windows, USB controller, optional USB Servo2040 connection
- **Production mode:** Linux (Raspberry Pi), Bluetooth controller, USB Servo2040 connection

Consider adding configuration or environment detection in `main.py` to handle differences (serial port paths, controller initialization, etc.).

## Future Considerations

For production deployment:

- Bluetooth pairing automation
- USB device permission setup
- Health monitoring/restart logic
- Log rotation for systemd journal

These can be added incrementally as you encounter issues.