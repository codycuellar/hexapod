#!/bin/bash
# Deployment script for updating Hexapod on Raspberry Pi
# This script syncs git changes, updates dependencies, and restarts the service

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

SERVICE_NAME="hexapod.service"

echo "=== Hexapod Deployment ==="
echo ""

# Check if repository directory exists
if [ ! -d "$REPO_DIR" ]; then
    echo "Error: Repository directory not found: $REPO_DIR"
    echo "Run scripts/setup-rpi.sh first for initial setup."
    exit 1
fi

cd "$REPO_DIR"
echo "Using repository directory: $REPO_DIR"

# Check if venv exists
if [ ! -d "venv" ]; then
    echo "Error: Virtual environment not found. Run scripts/setup-rpi.sh first."
    exit 1
fi

echo "Step 1: Pulling latest changes from git..."
git pull || {
    echo "Warning: git pull failed. Continuing with current code..."
}

echo ""
echo "Step 2: Updating Python dependencies..."
source venv/bin/activate
pip install --upgrade pip
pip install -e .[hexapod]

echo ""
echo "Step 3: Updating systemd service (if service file changed)..."
if [ -f "deploy/hexapod.service" ]; then
    SYSTEMD_USER_DIR="${HOME}/.config/systemd/user"
    mkdir -p "$SYSTEMD_USER_DIR"
    SERVICE_FILE="${SYSTEMD_USER_DIR}/${SERVICE_NAME}"

    # Replace %HOME% placeholder in service file template
    sed "s|%HOME%|${HOME}|g" "${REPO_DIR}/deploy/hexapod.service" > "$SERVICE_FILE"
    systemctl --user daemon-reload
    echo "Service file updated."
fi

echo ""
echo "Step 4: Restarting service..."
if systemctl --user is-active --quiet "$SERVICE_NAME"; then
    echo "Service is running, restarting..."
    systemctl --user restart "$SERVICE_NAME"
else
    echo "Service is not running, starting..."
    systemctl --user start "$SERVICE_NAME"
fi

echo ""
echo "=== Deployment Complete! ==="
echo ""
echo "Service status:"
systemctl --user status "$SERVICE_NAME" --no-pager -l

echo ""
echo "View logs with: journalctl --user -u hexapod -f"

