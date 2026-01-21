#!/bin/bash
# One-time setup script for Hexapod on Raspberry Pi
# This script installs dependencies, creates venv, and sets up systemd service

set -e  # Exit on any error

# Determine repository directory
# If run from within the repo, use current directory
# Otherwise, default to ~/hexapod
if [ -f "pyproject.toml" ] || [ -f "../pyproject.toml" ]; then
    # Try current directory first
    if [ -f "pyproject.toml" ]; then
        REPO_DIR="$(pwd)"
    # Try parent directory
    elif [ -f "../pyproject.toml" ]; then
        REPO_DIR="$(cd .. && pwd)"
    fi
else
    REPO_DIR="${HOME}/hexapod"
fi

VENV_DIR="${REPO_DIR}/venv"
SERVICE_NAME="hexapod.service"
SYSTEMD_USER_DIR="${HOME}/.config/systemd/user"
SERVICE_FILE="${SYSTEMD_USER_DIR}/${SERVICE_NAME}"

echo "=== Hexapod Raspberry Pi Setup ==="
echo ""

# Check if running on Linux/Raspberry Pi
if [[ "$OSTYPE" != "linux-gnu"* ]]; then
    echo "Warning: This script is designed for Linux/Raspberry Pi"
    echo "Detected OS: $OSTYPE"
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Check if repository directory exists
if [ ! -d "$REPO_DIR" ]; then
    echo "Error: Repository directory not found: $REPO_DIR"
    echo "Please clone the repository first:"
    echo "  git clone git@github.com:codycuellar/hexapod.git $REPO_DIR"
    exit 1
fi

cd "$REPO_DIR"
echo "Using repository directory: $REPO_DIR"

# Check if pyproject.toml exists
if [ ! -f "pyproject.toml" ]; then
    echo "Error: pyproject.toml not found. Are you in the right directory?"
    exit 1
fi

echo "Step 1: Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y python3 python3-venv python3-pip git

echo ""
echo "Step 2: Creating virtual environment..."
if [ -d "$VENV_DIR" ]; then
    echo "Virtual environment already exists, skipping..."
else
    python3 -m venv "$VENV_DIR"
    echo "Virtual environment created at $VENV_DIR"
fi

echo ""
echo "Step 3: Activating virtual environment and installing package..."
source "${VENV_DIR}/bin/activate"
pip install --upgrade pip
pip install -e .[hexapod]

echo ""
echo "Step 4: Adding user to dialout group (for serial port access)..."
if ! groups | grep -q dialout; then
    sudo usermod -aG dialout "$USER"
    echo "User added to dialout group. You may need to log out and back in for this to take effect."
else
    echo "User already in dialout group."
fi

echo ""
echo "Step 5: Enabling user service lingering (allows auto-start without login)..."
loginctl enable-linger "$USER" || {
    echo "Warning: Failed to enable lingering. Service may not start on boot without login."
    echo "You can try running this manually: loginctl enable-linger $USER"
}

echo ""
echo "Step 6: Installing systemd service..."
mkdir -p "$SYSTEMD_USER_DIR"

# Replace %HOME% placeholder in service file template
sed "s|%HOME%|${HOME}|g" "${REPO_DIR}/deploy/hexapod.service" > "$SERVICE_FILE"

echo "Service file installed to: $SERVICE_FILE"

echo ""
echo "Step 7: Enabling and starting service..."
systemctl --user daemon-reload
systemctl --user enable "$SERVICE_NAME"

echo ""
echo "=== Setup Complete! ==="
echo ""
echo "The hexapod service has been installed and enabled."
echo "It will automatically start on device boot."
echo ""
echo "Useful commands:"
echo "  Start service:    systemctl --user start hexapod"
echo "  Stop service:     systemctl --user stop hexapod"
echo "  Restart service:  systemctl --user restart hexapod"
echo "  Check status:     systemctl --user status hexapod"
echo "  View logs:        journalctl --user -u hexapod -f"
echo ""
read -p "Start the service now? (Y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Nn]$ ]]; then
    systemctl --user start "$SERVICE_NAME"
    echo "Service started!"
    sleep 2
    systemctl --user status "$SERVICE_NAME" --no-pager -l
fi

