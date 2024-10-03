#!/bin/bash

if [ $1 ]; then
    trap 'exit' INT
    for i in $(seq 0 $1);
    do
        python3 generate_ir_isaacsim.py $i
    done
else
    echo "Use this script to generate ir_maps between 0 and <num_maps> (inclusive)"
    echo "Usage: bash generate_ir.sh <num_ir_maps>"
fi


