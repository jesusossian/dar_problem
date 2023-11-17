# stdForm

module stdForm

using JuMP
using Gurobi
using CPLEX
using data
using parameters

mutable struct stdVars
  xp
  yp
  sp
  xr
  yr
  sr
end

export stdForm, stdVars

function stdForm(inst::instData, params::parameterData)

  N = inst.N
    
  SD = zeros(Int,N,N)
  SR = zeros(Int,N,N)
 
  for i=1:N
    SD[i,i] = inst.D[i]
    SR[i,i] = inst.R[i]
    for j=(i+1):N
      SD[i,j] = SD[i,j-1] + inst.D[j]
      SR[i,j] = SR[i,j-1] + inst.R[j]
    end
  end
  
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
    #set_optimizer_attribute(model,"CPX_PARAM_LPMETHOD ",0) # method used in root node
    set_optimizer_attribute(model,"CPX_PARAM_NODELIM",params.maxnodes) # MIP node limit
    set_optimizer_attribute(model,"CPX_PARAM_THREADS",1) # number of threads    
  else
    println("No solver selected")
    return 0
  end

  ### variables ###
  @variable(model, 0 <= xp[t=1:N] <= Inf)
  @variable(model, 0 <= xr[t=1:N] <= Inf)
  @variable(model, yp[t=1:N], Bin)
  @variable(model, yr[t=1:N], Bin)
  @variable(model, 0 <= sp[t=1:N] <= Inf)
  @variable(model, 0 <= sr[t=1:N] <= Inf)

  ### objective function ###
  @objective(
    model, Min, 
    sum(inst.PP[t]*xp[t] + inst.HP[t]*sp[t] + inst.FP[t]*yp[t] for t=1:N) + 
    sum(inst.PR[t]*xr[t] + inst.HR[t]*sr[t] + inst.FR[t]*yr[t] for t=1:N)
  )

  ### constraints ###
  @constraint(model, bal_p0, xp[1] + xr[1] - sp[1] == inst.D[1])
  @constraint(model, bal_p[t=2:N], sp[t-1] + xp[t] + xr[t] - sp[t] == inst.D[t])

  @constraint(model, bal_r0, -xr[1] - sr[1] == - inst.R[1])
  @constraint(model, bal_r[t=2:N], sr[t-1] - xr[t] - sr[t] == - inst.R[t])

  @constraint(model, set_xp[t=1:N], xp[t] <= SD[t,N]*yp[t])
  @constraint(model, set_xr[t=1:N], xr[t] <= min(SR[1,t],SD[t,N])*yr[t])

  @constraint(
    model, knap[t=1:N], xp[t] + xr[t] <= inst.C[t]
  )

  @constraint(model, sp[N] == 0)

  # inequalities

  #@constraint(
  #  model, vin01[t=1:N], 
  #  xp[t] <= min(inst.C[t],SD[t,N],inst.C[t])*yp[t]
  #)

  #@constraint(
  #  model, vin02[t=1:N], 
  #  xr[t] <= min(SR[1,t],SD[t,N],inst.C[t])*yr[t]
  #)

  #@constraint(
  #  model, vin03[t=2:N,l=t:N,t<=l], 
  #  sp[t-1] + sum(SD[k,l]*(yp[k]+yr[k]) for k=t:l) >= SD[t,l]
  #)

  #@constraint(
  #  model, vin04[t=1:N,l=t:N,t<=l], 
  #  sr[l] + sum(min(SR[t,k],inst.C[k])*yr[k] for k=t:l) >= SR[t,l]
  #)

  #@constraint(
  #  model, vin05[t=2:N,l=t:N,t<=l], 
  #  sp[t-1] + sum(min(SD[k,l],inst.C[k])*yp[k] for k=t:l) 
  #  + sum(min(SR[1,k],SD[k,l],inst.C[k])*yr[k] for k=t:l) >= SD[t,l]
  #)

  #@constraint(
  #  model, vin06[t=1:N,l=t:N,t<=l], 
  #  sr[l] + sum(min(SR[1,k],SD[k,l],inst.C[k])*yr[k] for k=t:l) >= SR[t,l]
  #)

  if params.method == "lp"
    #undo_relax = 
    relax_integrality(model)
  end

  #write_to_file(model,"1clsrp_std.lp")

  ### solving the problem ###
  optimize!(model)
  
  opt = 0
  if termination_status(model) == MOI.OPTIMAL    
    println("status = ", termination_status(model))
    opt = 1
  else
    println("status = ", termination_status(model))
  end
    
  ### get solutions ###
  
  if params.method == "mip"
    bestbound = objective_bound(model)
    numnodes = node_count(model)
    gap = MOI.get(model, MOI.RelativeGap())
  end
  bestsol = objective_value(model)
  time = solve_time(model)

  ### print solutions ###
  open("saida.txt","a") do f
    if params.method == "mip"
      write(f,"$(params.instName);$(params.form);$(params.method);$(bestbound);$(bestsol);$(gap);$(time);$(numnodes);$(opt)\n")
    else
      write(f,"$(params.instName);$(params.form);$(params.method);$(bestsol);$(time)\n")
    end
  end
  
end

end
