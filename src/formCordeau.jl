# formCordeau

module formCordeau

using JuMP
using Gurobi
using CPLEX
using data
using parameters
using Distances
using LinearAlgebra

mutable struct vars
    xp
    yp
    sp
    xr
    yr
    sr
end

export fCordeau, vars

function fCordeau(inst::instData, params::paramsData)

    n = inst.nn
    k = inst.K
    MRD = inst.MRD
    Q = inst.Q
    timeMRT = inst.timeMRT
  
    r = inst.nr
    println("num_nodes = ",n)    
    println("num_requests = ",r)
    println("num_vehicle = ",k)

    c = zeros(Float64,n,n)
    t = zeros(Float64,n,n)
    for i=1:n
        for j=1:n
            c[i,j] = euclidean(inst.cx[i],inst.cy[j])
            t[i,j] = euclidean(inst.cx[i],inst.cy[j])
        end
    end
  
    #for i=1:n
    #    for j=1:n
    #        println(c[i,j])
    #    end
    #end
      
    ### select solver and define parameters ###
    if params.solver == "gurobi"  
        model = Model(Gurobi.Optimizer)
        set_optimizer_attribute(model,"TimeLimit",params.maxtime) # time limit
        set_optimizer_attribute(model,"MIPGap",params.tolgap) # relative MIP optimality gap
        #set_optimizer_attribute(model,"NodeLimit",params.maxnodes) # MIP node limit
        #set_optimizer_attribute(model,"NodeMethod",0) # method used to solve MIP node relaxations
        #set_optimizer_attribute(model,"Method",-1) # method used in root node
        set_optimizer_attribute(model,"Threads",1) # number of threads
        #set_optimizer_attribute(model,"SolutionLimit", 1) # first viable solution
        # -1(default) to an automatic setting,  0 for off (0), 1 for conservative, 2 for aggressive
        #set_optimizer_attribute(model,"Presolve",0)
        # -1(default) chooses automatically, 0 to shut off cuts, 1 for moderate cut generation, 2 for aggressive, and 3 for very aggressive
        #set_optimizer_attribute(model,"Cuts",0) 
        set_optimizer_attribute(model,"NodefileStart",0.5)
        set_optimizer_attribute(model,"NodefileDir","/home/jossian/Downloads/")
        #set_optimizer_attribute(model,"MIPFocus", 3)
    elseif params.solver == "cplex"
        model = Model(CPLEX.Optimizer)
        set_optimizer_attribute(model,"CPX_PARAM_TILIM",params.maxtime) # time limit
        set_optimizer_attribute(model,"CPX_PARAM_EPGAP",params.tolgap) # relative MIP optimality gap
        #set_optimizer_attribute(model,"CPX_PARAM_LPMETHOD",0) # method used in root node
        set_optimizer_attribute(model,"CPX_PARAM_NODELIM",params.maxnodes) # MIP node limit
        set_optimizer_attribute(model,"CPX_PARAM_THREADS",1) # number of threads    
    else
        println("No solver selected")
        return 0
    end
    
    #ini = 1
    #fim = n
    #P = 2:17
    #D = 18:33 

    ### variables ###
    @variable(model, x[i=1:n,j=1:n], Bin)

    ### objective function ###
    @objective(
        model, Min, 
        sum(sum(sum(c[i,j]*x[i,j] for i=1:n) for j=1:n) for t=1:k)
    )

    ### constraints ###
    @constraint(model, serv_req[i=2:17],  sum(sum(x[i,j] for t=1:k) for j=1:n) == 1)

    #@constraint(model, bal_r[t=2:N], sr[t-1] - xr[t] - sr[t] == - inst.R[t])

    #@constraint(model, set_xr[t=1:N], xr[t] <= min(SR[1,t],SD[t,N])*yr[t])

    if params.method == "lp"
        relax_integrality(model)
    end

    write_to_file(model,"darp_cordeau.lp")

  ### solving the problem ###
#  optimize!(model)
  
#  opt = 0
#  if termination_status(model) == MOI.OPTIMAL    
#    println("status = ", termination_status(model))
#    opt = 1
#  else
#    println("status = ", termination_status(model))
#  end
    
  ### get solutions ###
  
#  if params.method == "mip"
#    bestbound = objective_bound(model)
#    numnodes = node_count(model)
#    gap = MOI.get(model, MOI.RelativeGap())
#  end
#  bestsol = objective_value(model)
#  time = solve_time(model)

  ### print solutions ###
#  open("saida.txt","a") do f
#    if params.method == "mip"
#      write(f,"$(params.instName);$(params.form);$(params.method);$(bestbound);$(bestsol);$(gap);$(time);$(numnodes);$(opt)\n")
#    else
#      write(f,"$(params.instName);$(params.form);$(params.method);$(bestsol);$(time)\n")
#    end
#  end
  
end

end
