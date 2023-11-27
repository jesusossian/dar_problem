from pathlib import Path
import os
import networkx as nx
import numpy as np
import gurobipy as gp
from gurobipy import GRB
import time as trun
import igraph as ig

def form_cordeau(method_,data_):

    file_in = open(data_)
	
    line = file_in.readline().split()
    K = int(line[0]) # number of vehicles
    N = int(line[1]) # number of nodes
    MRD = int(line[2])
    Q = int(line[3])
    timeMRT = int(line[4])
    
    print(K,N,MRD,Q,timeMRT)
      
    NN = 2*N+2
    
    lst_noMRT = []  # maximum ride times, L
    
    for t in range(0,NN):
        lst_noMRT.append(timeMRT)

    lst_id = [] # id nodes, id
    lst_x = [] # coord x, cx
    lst_y = [] # coord y, cy 
    lst_st = [] # service_time,  
    lst_d = [] # load 
    lst_stw = [] # start_tw 
    lst_etw = [] # end_tw
    
    for t in range(0,NN):
        line = file_in.readline().split()
        lst_id.append(int(line[0]))
        lst_x.append(float(line[1]))
        lst_y.append(float(line[2]))
        lst_st.append(int(line[3]))
        lst_d.append(int(line[4]))
        lst_stw.append(int(line[5]))
        lst_etw.append(int(line[6]))
        
    for t in range(0,NN):
        print(lst_id[t]," ",lst_x[t]," ",lst_y[t]," ",lst_st[t]," ",lst_d[t]," ",lst_stw[t]," ",lst_etw[t])
        
    file_in.close()

    cost = np.zeros((NN,NN),dtype=float)
    ctim = np.zeros((NN,NN),dtype=float)
    for i in range(0,NN):
        for j in range(0,NN):
            dist = np.linalg.norm(lst_x[i]-lst_y[j])
            print(dist)
            cost[i,j] = dist
            ctim[i,j] = dist

    #lb = np.zeros((N), dtype=float)
    #ub = np.zeros((N), dtype=float)
    #time = np.zeros((N), dtype=float)
    #gap = np.zeros((N), dtype=float)
    #nodes = np.zeros((N), dtype=float)
    #status = np.zeros((N), dtype=float)

    model = gp.Model(f"{data_}")

#    if (method_=="mip"):
#        x = model.addVars(N,vtype=GRB.BINARY,name="x")
#    else:
#        x = model.addVars(N,lb=0.0,ub=1.0,vtype=GRB.CONTINUOUS,name="x")
        
    # configurando parametros
    model.Params.TimeLimit = 3600
    model.Params.MIPGap = 1.e-6
    model.Params.Threads = 1
    model.Params.Presolve = 0
    #model.Params.Cuts = 0
 
    # Turn off display and heuristics
    #gp.setParam('OutputFlag', 0)
    #gp.setParam('Heuristics', 0)


    node = list(tuple())
    for t in range(0,K):
        for i in range(0, NN):
            for j in range(0, NN):
                node.append((i,j,t))

    x = model.addVars(node, vtype=GRB.BINARY, name='x')
    
    obj = 0
    for t in range(0,K):
        for i in range(0,NN):
            for j in range(0,NN):
                obj += cost[i][j]*x[i,j,t]
        
#    obj += 1 * x[j]
#    sum(sum(sum(cost[i][j]*x[i,j] for i=0:NN) for 0=1:NN) for t=0:K)
    
    model.setObjective(obj, GRB.MINIMIZE)

#            # geodesic
#            for u in G:
#                for w in G:
#                    #if dm[u,w] <= N:
#                    for s in G:
#                        if (s != u) and (s != w):
#                            if (dm[u,s] + dm[s,w] == dm[u,w]):
#                                model.addConstr(x[u] + x[w] >= x[s], "geo_c1_full")

#            #model.write(f"{instance_}_{i}.lp")

#            model.optimize()

#            tmp = 0
#            if model.status == GRB.OPTIMAL:
#                tmp = 1
# 
#            if method_ == "mip":
#                lb[i] = model.objBound
#                ub[i] = model.objVal
#                gap[i] = model.MIPGap
#                time[i] = model.Runtime
#                nodes[i] = model.NodeCount
#                status[i] = tmp
#            else:
#                ub[i] = model.objVal
#                time[i] = model.Runtime
#                status[i] = tmp

#            model.dispose()
            
#        # end tukey for node i
  
#    for i in G:
#        if (method_=="mip"):
#            arquivo = open(os.path.join(result_path,instance_),'a')
#            tmp = i
#            arquivo.write(
#                str(tmp)+';'
#                +str(round(lb[i],1))+';'
#                +str(round(ub[i],1))+';'
#                +str(round(gap[i],2))+';'
#                +str(round(time[i],2))+';'
#                +str(round(nodes[i],1))+';'
#                +str(round(status[i],1))+'\n'
#            )
#            arquivo.close()
#        else:
#            arquivo = open(os.path.join(result_path,instance_),'a')
#            tmp = i
#            arquivo.write(
#                str(tmp)+';'
#                +str(round(ub[i],1))+';'
#                +str(round(time[i],2))+';'
#                +str(round(status[i],1))+'\n'
#            )
#            arquivo.close()
