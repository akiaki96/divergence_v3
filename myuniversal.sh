#!/bin/bash

show_help() {
    cat << EOF
Usage: $0 [options]

Options:
  -b    Build
  -f    Flash
  -m    Monitor
  -h    Show help

No options:
  Build + Flash + Monitor
EOF
}

build=false
flash=false
monitor=false

if [ $# -eq 0 ]; then
    build=true
    flash=true
    monitor=true
fi

while getopts "bfmh" opt
do
    case $opt in
        b) build=true ;;
        f) flash=true ;;
        m) monitor=true ;;
        h)
            echo "Usage: $0 [-b] [-f] [-m]"
            exit 0
            ;;
    esac
done

if $build; then
    scripts/build.sh
fi

if $flash; then
    scripts/flash.sh
fi

if $monitor; then
    python tools/get_log.py
fi