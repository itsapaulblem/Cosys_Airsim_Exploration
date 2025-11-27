#!/bin/bash
# Terminator Multi-Drone Development Shortcuts

# Basic Terminator aliases
alias term='terminator --layout=default'
alias multi-drone='terminator-multi-drone'
alias drone-term='terminator --layout=default --title="Multi-Drone Development"'

# Enhanced terminal environment
export TERM=xterm-256color
export COLORTERM=truecolor

# Debug utilities (simple versions)
alias terminator-log='tail -f /home/Aortz/.terminator-autostart.log'
alias terminator-restart='pkill -f terminator; sleep 1; terminator-multi-drone &'
alias terminator-debug='terminator-config-debug'