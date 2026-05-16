#!/bin/bash

############################################################
# COLORS
############################################################

GREEN='\033[0;32m'
BLUE='\033[1;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

############################################################
# CHECKLIST UI FUNCTIONS
############################################################

step() {
    echo -e "${BLUE}[...]${NC} $1"
}

success() {
    echo -e "${GREEN}[✓]${NC} $1"
}

warn() {
    echo -e "${YELLOW}[!]${NC} $1"
}

fail() {
    echo -e "${RED}[✗]${NC} $1"
}

############################################################
# HEADER
############################################################

clear

echo "==========================================="
echo "         MIRA AUTOMATION STARTUP"
echo "==========================================="
echo

############################################################
# STEP 1
############################################################

step "Checking tmux installation"

if command -v tmux &> /dev/null; then
    success "tmux detected"
else
    fail "tmux not installed"
    exit 1
fi

############################################################
# STEP 2
############################################################

step "Checking sshpass installation"

if command -v sshpass &> /dev/null; then
    success "sshpass detected"
else
    fail "sshpass not installed"
    echo
    echo "Install using: sudo apt install sshpass"
    exit 1
fi

############################################################
# STEP 3
############################################################

step "Creating tmux sessions"

"$HOME/Desktop/mira_auto/mira_tmux.sh"

success "tmux sessions ready"

############################################################
# STEP 4
############################################################

step "Launching teleop window"

gnome-terminal -- tmux attach -t mira_teleop &

success "Teleop window launched"

############################################################
# STEP 5
############################################################

step "Attaching onboard session to current terminal"

success "Mira automation startup complete"

############################################################
# ATTACH CURRENT TERMINAL
############################################################

tmux attach -t mira_onboard