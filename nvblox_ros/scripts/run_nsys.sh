#!/usr/bin/bash
set -e

if [ $# == 0 ]
then
    echo "Run nsight-systems profiling with common settings. Default is to run a light profiling with little overhead. "
    echo " Set FULL=1 to run all-in profiling."
    echo "Usage: [DELAY=X] [DURATION=Y] [FULL=1] $0 command [args]"
    exit 1
fi

[[ -z $DELAY ]] && DELAY=30
[[ -z $DURATION ]] && DURATION=30

if [ -z $FULL ]
then
    # Run light profiling with little overhead
    set -x

    nsys profile -o nsys_light.nsys-rep \
         --force-overwrite=true \
         --delay=$DELAY \
         --duration=$DURATION \
         --stats=false \
         --sample=none \
         --backtrace=none \
         --cpuctxsw=none \
         --cudabacktrace=none \
         --trace=nvtx $@

else
    # Run all-in profiling

    # Backtrace source is platform dependent
    ARCHITECTURE=$(uname -m)
    if [ "$ARCHITECTURE" == "x86_64" ]; then
        echo "Running on x86_64"
        BACKTRACE_METHOD="lbr"
    elif [ "$ARCHITECTURE" == "aarch64" ]; then
        echo "Running on AARCH64"
        BACKTRACE_METHOD="dwarf"
    else
        echo "Unknown architecture: $ARCHITECTURE"
        exit 1
    fi
    echo "Backtrace method: $BACKTRACE_METHOD"

    set -x

    nsys profile -o nsys_full.nsys-rep \
         --force-overwrite=true \
         --delay=$DELAY \
         --duration=$DURATION \
         --stats=true \
         --sample=process-tree \
         --backtrace=$BACKTRACE_METHOD \
         --trace=nvtx,cuda \
         --samples-per-backtrace=2 \
         --cpuctxsw=process-tree \
         --trace-fork-before-exec=true \
         $@
fi
