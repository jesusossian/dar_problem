module data

#using Statistics
#using Random

struct instData
  K::Int # num_vehicles 
  N::Int # num_nodes
  MRD::Int # max_route_duration 
  Q::Int # veh_capacity 
  timeMRT::Int # temp_max_ride_time
  noMRT # nodes maximum ride times
  id # id node
  coord_x # coord x
  coord_y # coord y
  serv_time # service_time 
  demand # load 
  start_tw # start_tw 
  end_tw # end_tw
  num_requests # number of requests
end

export instData, readData

function readData(instFile,params)
  file = open(instFile)
  fileText = read(file, String)
  tokens = split(fileText) 
  #tokens will have all the tokens of the input file in a single vector. We will get the input token by token

  # read the problem's dimensions N
  aux = 1
  K = parse(Int,tokens[aux])
  aux = aux+1
  N = parse(Int,tokens[aux])
  aux = aux+1
  MRD = parse(Int,tokens[aux])
  aux = aux+1
  Q = parse(Int,tokens[aux])
  aux = aux+1
  timeMRT = parse(Int,tokens[aux])

  println(K)
  println(N)
  println(MRD)
  println(Q)
  println(timeMRT)

  #resize data structures according to N
  noMRT = zeros(Float64,N)  
  id = zeros(Int,N)
  coord_x = zeros(Float64,N)
  coord_y = zeros(Float64,N)
  serv_time = zeros(Float64,N) 
  demand = zeros(Float64,N)
  start_tw = zeros(Float64,N) 
  end_tw = zeros(Float64,N)

  for i=1:N
    noMRT[i] = timeMRT
  end
  
  for i=1:N
    aux = aux+1
    id[i] = parse(Int,tokens[aux])
    aux = aux+1
    coord_x[i] = parse(Float64,tokens[aux])
    aux = aux+1
    coord_y[i] = parse(Float64,tokens[aux])
    aux = aux+1
    serv_time[i] = parse(Float64,tokens[aux]) 
    aux = aux+1
    demand[i] = parse(Float64,tokens[aux])
    aux = aux+1
    start_tw[i] = parse(Float64,tokens[aux]) 
    aux = aux+1
    end_tw[i] = parse(Float64,tokens[aux])
  end
  
  num_requests = N
  
  println(num_requests)

  
  close(file)

  inst = instData(K,N,MRD,Q,timeMRT,noMRT,id,coord_x,coord_y,serv_time,demand,start_tw,end_tw,num_requests)

  return inst

end

end
