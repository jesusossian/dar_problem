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
import formCordeau

params = parameters.readParams(ARGS)

#julia 1clsr.jl --inst instancia --form ${form} 

# read instance data
inst = data.readData(params.instName, params)

#if (params.form == "cordeau")
formCordeau.fCordeau(inst, params)
#end

