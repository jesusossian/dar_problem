module data

#using Statistics
#using Random

mutable struct instData
    K::Int # num_vehicles 
    N::Int # if cordeau num_users/requests, if gaul num_nodes 
    MRD::Int # max_route_duration 
    Q::Int # veh_capacity 
    timeMRT::Int # temp_max_ride_time
    noMRT # nodes maximum ride times
    id # id node
    cx # coord x
    cy # coord y
    st # service_time 
    d # load 
    stw # start_tw 
    etw # end_tw
    nr # number of requests
    nn # number of nodes
end

export instData, readData

function readData(instFile,params)
    file = open(instFile)
    fileText = read(file, String)
    tokens = split(fileText) 
    #tokens will have all the tokens of the input file in a single vector. We will get the input token by token

    #read the problem's dimensions N
    aux = 1
    K = parse(Int,tokens[aux]) # number of vehicles
    aux = aux+1
    N = parse(Int,tokens[aux]) # number of nodes
    aux = aux+1
    MRD = parse(Int,tokens[aux]) # maximum route duration, T_k
    aux = aux+1
    Q = parse(Int,tokens[aux]) # vehicle capacity, Q_k 
    aux = aux+1
    timeMRT = parse(Int,tokens[aux]) # maximum ride time

    #println(K)
    #println(N)
    #println(MRD)
    #println(Q)
    #println(timeMRT)

    nn = 2*N+2
    #resize data structures according to N
    noMRT = zeros(Float64,nn)  # maximum ride times, L
    id = zeros(Int,nn) # id nodes, id
    cx = zeros(Float64,nn) # coord x, cx
    cy = zeros(Float64,nn) # coord y, cy 
    st = zeros(Float64,nn) # service_time,  
    d = zeros(Float64,nn) # load 
    stw = zeros(Float64,nn) # start_tw 
    etw = zeros(Float64,nn) # end_tw

    for i=1:nn
        noMRT[i] = timeMRT
    end
  
    for i=1:nn
        aux = aux+1
        id[i] = parse(Int,tokens[aux])
        aux = aux+1
        cx[i] = parse(Float64,tokens[aux])
        aux = aux+1
        cy[i] = parse(Float64,tokens[aux])
        aux = aux+1
        st[i] = parse(Float64,tokens[aux]) 
        aux = aux+1
        d[i] = parse(Float64,tokens[aux])
        aux = aux+1
        stw[i] = parse(Float64,tokens[aux]) 
        aux = aux+1
        etw[i] = parse(Float64,tokens[aux])
    end
  
    nr = N
  
    #println(num_requests)
  
    close(file)

    inst = instData(K,N,MRD,Q,timeMRT,noMRT,id,cx,cy,st,d,stw,etw,nr,nn)

    return inst

end

end
