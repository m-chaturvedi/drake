#!/bin/bash
me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
export LD_LIBRARY_PATH=/home/chaturvedi/workspace/libcxx_msan/lib
export MSAN_OPTIONS="$MSAN_OPTIONS:suppressions=$mydir/msan.supp"
"$@"
