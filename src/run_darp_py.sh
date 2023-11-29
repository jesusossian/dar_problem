#!/bin/bash

method_=mip
form_=cordeau

mkdir -p ../result
#mkdir -p ${inst_}s/${inst_}s${dim_}

for veh_ in a2
do
    for no_ in 16
    do
        python3 darp.py ../instances/darp_bc/${veh_}-${no_}.txt ${method_} ${veh_}_${no_} ${form_}
    done
done
