module data

#using Statistics
#using Random

struct instData
  K::Int # num_vehicles 
  N::Int # num_nodes
  MRD::Int # max_route_duration 
  Q::Int # veh_capacity 
  TMRT::Int # temp_max_ride_time
  NMRT # nodes maximum ride times
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
  TMRT = parse(Int,tokens[aux])

  println(K)
  println(N)
  println(MRD)
  println(Q)
  println(TMRT)

  #resize data structures according to N
  NMRT = zeros(Float64,N)
  #D = zeros(Int,N)

  for t=1:N
    NMRT[t] = TMRT
  end
  
  #aux = aux+1
  #FR[1] = parse(Float64,tokens[aux])
  #for t=2:N
  #  FR[t] = FR[1]
  #end
  
  #for t in 1:N
  #  aux = aux+1
  #  D[t] = parse(Int,tokens[aux])
  #end

  close(file)

  inst = instData(K,N,MRD,Q,TMRT,NMRT)

  return inst

end

end
