push!(LOAD_PATH, "src/")
# push!(DEPOT_PATH, JULIA_DEPOT_PATH)

#using Pkg
#Pkg.activate(".")
#Pkg.instantiate()
#Pkg.build()

using JuMP
using Gurobi
using CPLEX

import data
import parameters
#import stdForm

params = parameters.readParam(ARGS)

#julia 1clsr.jl --inst instancia --form ${form} 

# read instance data
inst = data.readData(params.instName, params)

#if (params.form == "std")
#    stdFormulation.stdForm(inst, params)
#end

