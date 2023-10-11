module parameters

struct paramData
  instName::String
  form::String ### std mc sp
  solver::String
  method::String
  maxtime::Int
  tolgap::Float64
  disablesolver::Int
  maxnodes::Int

end

export paramData, readParam

function readParam(ARGS)

  ### Set standard values for the parameters ###
  instName = "/home/jossian/repository/dar_problem/instances/darp_bc/a2-16.txt"
  form = "stdCP"
  solver = "gurobi"
  method = "mip"
  maxtime = 120
  tolgap = 1e-6
  disablesolver = 0
  maxnodes = 10000000.0


  ### Read the parameters and setvalues whenever provided ###
  for param in 1:length(ARGS)
    if ARGS[param] == "--inst"
      instName = ARGS[param+1]
      param += 1
    elseif ARGS[param] == "--form"
      form = ARGS[param+1]
      param += 1
    elseif ARGS[param] == "--solver"
      solver = ARGS[param+1]
      param += 1
    elseif ARGS[param] == "--method"
      method = ARGS[param+1]
      param += 1
    elseif ARGS[param] == "--maxtime"
      maxtime = parse(Int,ARGS[param+1])
      param += 1
    elseif ARGS[param] == "--tolgap"
      tolgap = parse(Float64,ARGS[param+1])
      param += 1
    elseif ARGS[param] == "--disablesolver"
      disablesolver = parse(Int,ARGS[param+1])
      param += 1
    elseif ARGS[param] == "--maxnodes"
      maxnodes = parse(Float64,ARGS[param+1])
      param += 1
    end
  end

  params = paramData(
    instName,
    form,
    solver,
    method,
    maxtime,
    tolgap,
    disablesolver,
    maxnodes
  )

  return params

end 

end
