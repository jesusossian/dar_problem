from pathlib import Path
import os
import networkx as nx
import numpy as np
import gurobipy as gp
from gurobipy import GRB
import time as trun
import igraph as ig
import math

def form_cordeau(method_,data_,out_path_,instance_):

    file_in = open(data_)
	
    line = file_in.readline().split()
    K = int(line[0]) # number of vehicles
    N = int(line[1]) # number of nodes
    max_rou_dur = int(line[2]) # maximum route duration for vehicle, T
    veh_cap = int(line[3]) # vehicles capacity, Q_k
    max_rid_time = int(line[4]) # maximum ride time of user, L
    
    # total duration of its route cannot exceed T_k
    
    print(K,N,max_rou_dur,veh_cap,max_rid_time)
 
    veh_capk = []  # vehicle capacity, Q_k
    for t in range(0,K):
        veh_capk.append(veh_cap)

    Tk = []  # maximum route duration, T_k    
    for t in range(0,K):
        Tk.append(max_rou_dur)
    
    Li = []  # maximum ride times, L_i    
    for i in range(0,2*(N+1)):
        Li.append(max_rid_time)

    idn = [] # id nodes, id
    cox = [] # coord x, cx
    coy = [] # coord y, cy 
    serv_time = [] # service_time, d_i, service duration
    load = [] # load, q_i, demand
    start_tw = [] # start_tw, e_i 
    end_tw = [] # end_tw, l_i
    
    for t in range(0,2*(N+1)):
        line = file_in.readline().split()
        idn.append(int(line[0]))
        cox.append(float(line[1]))
        coy.append(float(line[2]))
        serv_time.append(int(line[3]))
        load.append(int(line[4]))
        start_tw.append(int(line[5]))
        end_tw.append(int(line[6]))
        
    #for t in range(0,2*(N+1)):
    #    print(idn[t]," ")
    #    print(cox[t]," ")
    #    print(coy[t]," ")
    #    print(serv_time[t]," ")
    #    print(load[t]," ")
    #    print(start_tw[t]," ")
    #    print(end_tw[t])
        
    file_in.close()

    route_cost = np.zeros((2*(N+1),2*(N+1)),dtype=float)
    travel_time = np.zeros((2*(N+1),2*(N+1)),dtype=float)
    for i in range(0,2*N+1):
        for j in range(0,2*N+1):
            #dist = np.linalg.norm(cox[i]-coy[j])
            dist = (cox[i]-cox[j])*(cox[i]-cox[j]) + (coy[i]-coy[j])*(coy[i]-coy[j])
            dist = math.sqrt(dist)
            #print(dist)
            dist = round(dist,2)
            route_cost[i,j] = dist
            travel_time[i,j] = dist
            print(route_cost[i,j],end=" ")
        print("\n")

    time_win = np.zeros((2*(N+1)),dtype=float)   # time windows     
    for i in range(0,2*(N+1)):
        time_win[i] = end_tw[i] - start_tw[i]
        print(time_win[i])

    M = np.zeros((2*(N+1),2*(N+1),K), dtype=float)
    W = np.zeros((2*(N+1),2*(N+1),K), dtype=float)

    model = gp.Model(f"{data_}")
        
    # configurando parametros
    model.Params.TimeLimit = 3600
    model.Params.MIPGap = 1.e-6
    model.Params.Threads = 1
    model.Params.Presolve = 0
    #model.Params.Cuts = 0
 
    # Turn off display and heuristics
    #gp.setParam('OutputFlag', 0)
    #gp.setParam('Heuristics', 0)

    tuple3 = list(tuple())
    for i in range(0,2*(N+1)):
        for j in range(0,2*(N+1)):
            for k in range(0,K):
                tuple3.append((i,j,k))

    tuple2 = list(tuple())
    for i in range(0,2*(N+1)):
        for k in range(0,K):
            tuple2.append((i,k))
            
    
    # x_{i,j,k} = 1 if and only if arc (i,j) is travesed by vehicle k \in K
    x = model.addVars(tuple3,lb=0.0,ub=1.0,vtype=GRB.BINARY,name="x")

    # u_{i,k}, time at which vehicle k starts servicing at vertex i 
#    u = model.addVars(tuple2,vtype=GRB.CONTINUOUS,name="u")
    B_ini = {}
    for k in range(0,K):
        B_ini[k] = model.addVar(vtype=GRB.CONTINUOUS,lb=start_tw[0],ub=end_tw[0], name="B_ini(%s)"%k)

    B_end = {}
    for k in range(0,K):
        B_end[k] = model.addVar(vtype=GRB.CONTINUOUS,lb=start_tw[0],ub=end_tw[0], name="B_end(%s)"%k)

    B = {}
    for i in range(1,2*N+1):
        B[i] = model.addVar(vtype=GRB.CONTINUOUS,lb=start_tw[i],ub=end_tw[i], name="B(%s)"%i)
        
    # r_{i,k}, ride time of use i (corresponding to request (i,n+i) on vehicle k)
#    r = model.addVars(tuple2,vtype=GRB.CONTINUOUS,name="r")
    L = {}
    for i in range(1,N+1):
        L[i] = model.addVar(vtype=GRB.CONTINUOUS,lb=travel_time[i,N+i],ub=Li[i], name="L(%s)"%i)

    # w_{i,k}, load of vehicle k upon leaving vertex i
#    w = model.addVars(tuple2,vtype=GRB.CONTINUOUS,name="w")
    Q_ini = {}
    for k in range(0,K):
        Q_ini[k] = model.addVar(vtype=GRB.CONTINUOUS,lb=0,ub=veh_cap,name="Q_ini(%s)"%k)

    Q_end = {}
    for k in range(0,K):
        Q_end[k] = model.addVar(vtype=GRB.CONTINUOUS,lb=0,ub=veh_cap,name="Q_end(%s)"%k)

    Q = {}
    for i in range(1,2*N+1):
        Q[i] = model.addVar(vtype=GRB.CONTINUOUS,lb=max(0,load[i]),ub=min(veh_cap,veh_cap+load[i]),name="Q(%s)"%i)
    
    obj = 0
    for i in range(0,2*N+1):
        # j \in {0, 2n}
        for j in range(0,2*N+1):
            for k in range(0,K):
                obj += route_cost[i][j]*x[i,j,k]
        # j = 2n+1   
        for k in range(0,K):
            obj += route_cost[i,0] * x[i,2*N+1,k]
            
    # i = 2n+1
    # j \in {0, 2n}
    for j in range(0,2*N+1) :
        for k in range(0,K):
            obj += route_cost[0,j] * x[2*N+1,j,k]
        
    model.setObjective(obj, GRB.MINIMIZE)
    
    # P = { 1, ..., N }
    # D = { N+1, ..., 2*N }
    # 0, 2*N+1
    
    for k in range(0,K):
        # x_{i,i} = 0 for all i
        for i in range(0,2*N+2):
            model.addConstr(x[i,i,k] == 0)
            
        #// x_{i,0} = 0 for all i \in P U D and i = 2*n+1
        for i in range(1,2*N+2):
            model.addConstr(x[i,0,k] == 0)

        # x_{2*n+1,i} = 0 for all i \in P U D and i = 0
        for i in range(0,2*N+1):
            model.addConstr(x[2*N+1,i,k] == 0)

        # x_{i,2*n+1} = 0 for all i \in P 
        # x_{n+i,i} = 0 for all i \in P 
        for i in range(1,N+1):
            model.addConstr(x[i,2*N+1,k] == 0)
            model.addConstr(x[N+i,i,k] == 0)

        # x_{0,i} = 0 for all i \in D
        for i in range(N+1,2*N+1):
            model.addConstr(x[0,i,k] == 0)
            
    # pick_up - each user is picked up by exactly one vehicle and then travels to exactly one different node
    for i in range(1,N+1):
        for j in range(0,2*N+2):
            if (j != i):
                for k in range(0,K):
                    model.addConstr(x[i,j,k] == 1)
                    
    # drop_off - everyone who is picked up must also be dropped off again (by the same vehicle)
    for k in range(0,K):
        for i in range(1,N+1):
        
            con1 = 0
            for j in range(0,2*N+2):
                if (j!=i):
                    #expr1 += x[i][j][k];
                    con1 += x[i,j,k]
                    
            con2 = 0
            for j in range(0,2*N+2):
                if (j!=N+i):
                    #expr2 += x[n+i][j][k];
                    con2 += x[N+i,j,k]
                    
            model.addConstr(con1 - con2 == 0)
            
    
    # leave_depot - every vehicle leaves the depot
    for k in range(0,K):
        expr = 0
        for j in range(1,2*N+2):
            expr += x[0,j,k];
        model.addConstr(expr == 1)

    # flow preservation
    for k in range(0,K):
        for i in range(1,2*N+1):
            expr = 0
            for j in range(0,2*N+2):
                if (j!=i):
                    expr += x[j,i,k] - x[i,j,k]
            model.addConstr(expr == 0)
            

    # return_depot
    for k in range(0,K):
        expr = 0
        for i in range(0,2*N+1):
            expr += x[i,2*N+1,k]
        model.addConstr(expr == 1)
        
        
#        for (int k=0; k<D.num_vehicles; k++)
#        {
#            for (int j=1; j<=2*n; j++)
#            {
#                expr = -B[j] + B_0[k] + D.nodes[0].service_time + D.d[0][j] - DARPH_MAX(0, D.nodes[0].end_tw + D.nodes[0].service_time + D.d[0][j] - D.nodes[j].start_tw) * (1 - x[0][j][k]);
#                model.add(IloRange(env,expr,0));
#                expr.clear();
#            }
#        }

#        for (int i=1; i<=2*n; i++)
#        {
#            for (int j=1; j<=2*n; j++)
#            {
#                if (j!=i)
#                {
#                    for (int k=0; k<D.num_vehicles; k++)
#                    {
#                        expr2 += x[i][j][k];
#                    }
#                    expr = -B[j] + B[i] + D.nodes[i].service_time + D.d[i][j] - DARPH_MAX(0, D.nodes[i].end_tw + D.nodes[i].service_time + D.d[i][j] - D.nodes[j].start_tw) * (1 - expr2);
#                    model.add(IloRange(env,expr,0));
#                    expr.clear();
#                    expr2.clear();
#                }
#            }
#        }

#        for (int k=0; k<D.num_vehicles; k++)
#        {
#            for (int i=1; i<=2*n; i++)
#            {
#                expr = -B_2n1[k] + B[i] + D.nodes[i].service_time + D.d[i][0] - DARPH_MAX(0, D.nodes[i].end_tw + D.nodes[i].service_time + D.d[i][0] - D.nodes[0].start_tw) * (1 - x[i][2*n+1][k]);
#                model.add(IloRange(env,expr,0));
#                expr.clear();
#            }
#        }
        
#        for (int k=0; k<D.num_vehicles; k++)
#        {
#            expr = -B_2n1[k] + B_0[k] + D.nodes[0].service_time + D.d[0][0] - DARPH_MAX(0, D.nodes[0].end_tw + D.nodes[0].service_time + D.d[0][0] - D.nodes[0].start_tw) * (1-x[0][2*n+1][k]);
#            model.add(IloRange(env,expr,0));
#            expr.clear();
#        }


    
    # constraint 2
#    for i in range(1,N+1):
#        con = 0
#        for t in range(0,K):
#            for j in range(0,2*(N+1)):
#                con += x[i,j,t]
#        model.addConstr(con == 1)

    # constraint 2
#    for t in range(0,K):
#        con = 0
#        for i in range(0,2*(N+1)):
#            con += x[0,i,t]
#        model.addConstr(con == 1)

    # constraint 3
#    for t in range(0,K):
#        con = 0
#        for i in range(0,2*(N+1)):
#            con += x[i,2*N+1,t]
#        model.addConstr(con == 1)

    # constraint 4
#    for i in range(1,N+1):
#        for t in range(0,K):
#            con = 0
#            for j in range(0,2*(N+1)):
#                con += x[i,j,t]
#            for j in range(0,2*(N+1)):
#                con -= x[N+i,j,t]
#            model.addConstr(con == 0)


    # constraint 5
#    for i in range(1,2*N+1):
#        for t in range(0,K):
#            con = 0
#            for j in range(0,2*(N+1)):
#                con += x[j,i,t]
#            for j in range(0,2*(N+1)):
#                con -= x[i,j,t]
#            model.addConstr(con == 0)
        
    # constraint 6
#    for i in range(0,2*(N+1)):
#        for j in range(0,2*(N+1)):
#            for t in range(0,K):
#                model.addConstr(u[j,t] >= (u[i,t] + serv_time[i] + travel_time[i,j])*x[i,j,t])

    # constraint 7
#    for i in range(0,2*(N+1)):
#        for j in range(0,2*(N+1)):
#            for t in range(0,K):
#                model.addConstr(w[j,t] >= (w[i,t] + load[j])*x[i,j,t])

    # constraint 8
#    for i in range(1,N+1):
#        for t in range(0,K):
#            model.addConstr(r[i,t] == u[N+i,t] - (u[i,t] + serv_time[i]))

    # constraint 9
#    for t in range(0,K):
#        model.addConstr(u[2*N+1,t] - u[0,t] <= Tk[t])

    # constraint 10_1
#    for i in range(0,2*(N+1)):
#        for t in range(0,K):
#            model.addConstr(u[i,t] >= start_tw[i])

    # constraint 10_2
#    for i in range(0,2*(N+1)):
#        for t in range(0,K):
#            model.addConstr(u[i,t] <= end_tw[i])

    # constraint 11_1
#    for i in range(1,N+1):
#        for t in range(0,K):
#            model.addConstr(r[i,t] >= travel_time[i,N+i])

    # constraint 11_2
#    for i in range(1,N+1):
#        for t in range(0,K):
#            model.addConstr(r[i,t] <= maxRidTime)

    # constraint 12_1
#    for i in range(0,2*(N+1)):
#        for t in range(0,K):
#            model.addConstr(max(0,load[i]) <= w[i,t])

    # constraint 12_1
#    for i in range(0,2*(N+1)):
#        for t in range(0,K):
#            model.addConstr(w[i,t] <= min(vehCapk[t],vehCapk[t] + load[i]))

    # constraint 15
#    for i in range(0,2*(N+1)):
#        for j in range(0,2*(N+1)):
#            for t in range(0,K):
#                M[i,j,t] >= max(0,end_tw[i] + serv_time[i] + travel_time[i,j] - start_tw[j])

#    for i in range(0,2*(N+1)):
#        for j in range(0,2*(N+1)):
#            for t in range(0,K):
#                model.addConstr(
#                    B[j,t] >= B[i,t] + serv_time[i] + travel_time[i,j] - M[i,j,t]*(1 - x[i,j,t])
#                )

    # constraint 16
#    for i in range(0,2*(N+1)):
#        for j in range(0,2*(N+1)):
#            for t in range(0,K):
#                W[i,j,t] >= min(vehCapk[t], vehCapk[t] + load[t])

#    for i in range(0,2*(N+1)):
#        for j in range(0,2*(N+1)):
#            for t in range(0,K):
#                model.addConstr(vehCapk[j,t] <= vehCapk[i,t] + load[j] - W[i,j,t]*(1 - x[i,j,t]))
                   

    model.write(f"dar_problem.lp")

    model.optimize()

#    tmp = 0
#    if model.status == GRB.OPTIMAL:
#        tmp = 1
     
#    if method_ == "mip":
#        lb = model.objBound
#        ub = model.objVal
#        gap = model.MIPGap
#        time = model.Runtime
#        nodes = model.NodeCount
#        status = tmp
#    else:
#        ub= model.objVal
#        time = model.Runtime
#        status = tmp
  
#    if (method_=="mip"):
#        arquivo = open(os.path.join(out_path_,instance_),'a')
#        arquivo.write(
#            str(tmp)+';'
#            +str(round(lb,1))+';'
#            +str(round(ub,1))+';'
#            +str(round(gap,2))+';'
#            +str(round(time,2))+';'
#            +str(round(nodes,1))+';'
#            +str(round(status,1))+'\n'
#        )
#        arquivo.close()
#    else:
#        arquivo = open(os.path.join(out_path_,instance_),'a')
#        arquivo.write(
#            str(tmp)+';'
#            +str(round(ub[i],1))+';'
#            +str(round(time[i],2))+';'
#            +str(round(status[i],1))+'\n'
#        )
#        arquivo.close()
