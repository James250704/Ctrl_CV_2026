#!/bin/bash
export HOME=/sessions/eloquent-awesome-pasteur
export PATH=$HOME/.local/bin:/usr/bin:/bin
cd /sessions/eloquent-awesome-pasteur/mnt/Ctrl_CV_2026/
pio run -e ultrasonic -v > /sessions/eloquent-awesome-pasteur/mnt/Ctrl_CV_2026/build_result.txt 2>&1
echo "EXIT_CODE=$?" >> /sessions/eloquent-awesome-pasteur/mnt/Ctrl_CV_2026/build_exit_code.txt
