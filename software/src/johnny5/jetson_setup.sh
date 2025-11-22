#!/bin/bash

#############################################################################
# Johnny 5 Jetson Orin Setup Script
# Adapted from WhoAmI v2.0 for Johnny 5 (LeRobot + Differential Drive)
#############################################################################

set -e  # Exit on error
set -o pipefail

# Script metadata
VERSION="1.0.0"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_FILE="$HOME/johnny5_setup_$(date +%Y%m%d_%H%M%S).log"

# Configuration
PYTHON_MIN_VERSION="3.8"
VENV_PATH="$HOME/johnny5_env"

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log() {
    local level=$1
    shift
    local message="$@"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$timestamp] [$level] $message" | tee -a "$LOG_FILE"
}

print_success() { echo -e "${GREEN}✓ $@${NC}"; log "SUCCESS" "$@"; }
print_error() { echo -e "${RED}✗ $@${NC}"; log "ERROR" "$@"; }
print_warning() { echo -e "${YELLOW}⚠ $@${NC}"; log "WARNING" "$@"; }
print_info() { echo -e "${BLUE}ℹ $@${NC}"; log "INFO" "$@"; }

check_permissions() {
    if [ "$EUID" -eq 0 ]; then
        print_error "Please do not run as root. Script will request sudo when needed."
        exit 1
    fi
}

install_dependencies() {
    print_info "Installing System Dependencies..."
    sudo apt update
    sudo apt install -y \
        build-essential cmake git python3-pip python3-dev python3-venv \
        libopencv-dev python3-opencv \
        portaudio19-dev espeak flac alsa-utils \
        libusb-1.0-0-dev libudev-dev \
        libopenblas-dev liblapack-dev gfortran \
        libhdf5-dev libhdf5-serial-dev \
        jetson-stats

    # Add user to dialout group for serial access (Base/Gimbal)
    sudo usermod -a -G dialout $USER
    sudo usermod -a -G video $USER
}

setup_python_env() {
    print_info "Setting up Python Virtual Environment..."
    if [ ! -d "$VENV_PATH" ]; then
        python3 -m venv "$VENV_PATH"
    fi
    
    source "$VENV_PATH/bin/activate"
    pip install --upgrade pip wheel setuptools
    
    print_info "Installing Python Packages..."
    # Core Robotics
    pip install numpy==1.23.5  # Jetson compat
    pip install opencv-python
    pip install pyserial
    
    # AI / Vision (WhoAmI stack)
    pip install face_recognition
    pip install depthai  # OAK-D
    pip install cryptography  # Gun.js encryption
    
    # Voice
    pip install pyttsx3
    pip install SpeechRecognition
    
    # LeRobot (from source usually, but install deps here)
    pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
    pip install lerobot
}

setup_udev_rules() {
    print_info "Configuring UDEV rules for OAK-D and Serial..."
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/80-movidius.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
}

main() {
    check_permissions
    install_dependencies
    setup_udev_rules
    setup_python_env
    
    print_success "Johnny 5 Setup Complete!"
    print_info "Activate environment with: source $VENV_PATH/bin/activate"
}

main "$@"