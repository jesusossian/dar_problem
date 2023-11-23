#!/bin/bash

#path_=k_cluster 
#inst_=kcluster
#dim_=80
method_=mip 

#mkdir -p ${inst_}s/${inst_}s${dim_}

for veh_ in a2
do
    for no_ in 16
    do
        python3 darp.py ../instances/darp_bc/${veh_}-${no_}.txt ${method_}
    done
done
