#include "DARPH.h"

template <int Q>
bool EBMILP<Q>::solve_furtado(DARP& D, DARPGraph<Q>& G) {
    // measure model and solve time
    const auto before = clock::now();
    
    bool solved = false;
   
    // We use this stringstream to create variable and constraint names
    std::stringstream name;

    // create graph
    check_paths(D);
    G.create_graph(D,f); 
    create_maps(D, G);

    IloEnv env;
    
    try { 
        IloModel model(env);
        
        // Variables
        IloArray<IloNumVarArray> x(env,2*n+2); // nodes 0, 2n+1
        // initialize 
        for (int i = 0; i <= 2*n+1; i++) {
            x[i] = IloNumVarArray(env,2*n+2);
            for (int j = 0; j <= 2*n+1; j++) {
                name << "x_{" << i << "," << j << "}";
                x[i][j] = IloNumVar(env, 0, 1, ILOBOOL, name.str().c_str());
                name.str("");  
            }
        }
        
        IloNumVarArray QQ(env,2*n+2);
        for (int i=0; i<=2*n; i++) {
            name << "Q_" << i;
            QQ[i] = IloNumVar(env, max(0, D.nodes[i].demand), min(D.veh_capacity, D.veh_capacity + D.nodes[i].demand), ILOFLOAT, name.str().c_str());
            name.str("");
        }
        name << "Q_33";
        QQ[2*n+1] = IloNumVar(env, max(0, D.nodes[0].demand), min(D.veh_capacity, D.veh_capacity + D.nodes[0].demand), ILOFLOAT, name.str().c_str());
        name.str("");


        IloNumVarArray B(env,2*n+2);
        for (int i=0; i<=2*n+1; i++) {
            name << "B_" << i;
            B[i] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, name.str().c_str());
            name.str("");
        }

        IloNumVarArray v(env,2*n+1);
        for (int i=1; i<=2*n; i++) {
            name << "v_" << i;
            v[i] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, name.str().c_str());
            name.str("");
        }

        IloExpr obj1(env); // routing costs
        
        // Create objective function
        for (int i = 0; i <= 2*n; i++) {   
            // j \in {0, 2n}
            for (int j = 0; j <= 2*n; j++) {
                obj1 += D.d[i][j] * x[i][j];  
            } 
            // j = 2n+1   
            obj1 += D.d[i][0] * x[i][2*n+1];
        }
        // i = 2n+1
        // j \in {0, 2n}
        for (int j = 0; j <= 2*n; j++) {
            obj1 += D.d[0][j] * x[2*n+1][j];  
        } 
        // i = 2n+1, j = 2n+1
        obj1 += D.d[0][0] * x[2*n+1][2*n+1]; 
        
        IloObjective obj(env, obj1, IloObjective::Minimize);
        model.add(obj);
        


        // trivial variable fixings
        // x_{i,i} = 0 for all i
        for (int i=0; i<=2*n+1; i++)
        {
            model.add(IloRange(env,0,x[i][i],0));
        }
        // x_{i,0} = 0 for all i \in P U D and i = 2*n+1
        for(int i=1; i<=2*n+1; i++)
        {
            model.add(IloRange(env,0,x[i][0],0));
        }
        // x_{2*n+1,i} = 0 for all i \in P U D and i = 0
        for (int i=0; i<=2*n; i++)
        {
            model.add(IloRange(env,0,x[2*n+1][i],0));
        }
        
        // x_{i,2*n+1} = 0 for all i \in P 
        // x_{n+i,i} = 0 for all i \in P 
        for (int i=1; i<=n; i++)
        {
            model.add(IloRange(env,0,x[i][2*n+1],0));
            model.add(IloRange(env,0,x[n+i][i],0));
        }
        // x_{0,i} = 0 for all i \in D
        for (int i = n+1; i<=2*n; i++)
        {
            model.add(IloRange(env,0,x[0][i],0));
        }
        
        for (int i=1; i<=n; i++)
        {
            for (int j=1; j<=n; j++)
            {
                if (j != i)
                {
                    // x_{i,j} = 0
                    if (f[j][i][0] == 0 && f[j][i][1] == 0)
                    {
                        model.add(IloRange(env,0,x[i][j],0));
                    }
                    // x_{n+i,n+j} = 0
                    if (f[j][i][0] == 0 && f[i][j][1] == 0)
                    {
                        model.add(IloRange(env,0,x[n+i][n+j],0));
                    }
                    // x_{i,n+j} = 0
                    if (f[i][j][0] == 0)
                    {
                        model.add(IloRange(env,0,x[i][n+j],0));
                    }
                } 
            }
        }
        


        // Constraints
        IloExpr expr(env);

        for (int j=1; j<=2*n; j++)
        {
            for (int i=0; i<=2*n+1; i++)
            {
                expr += x[i][j];
            }
            model.add(IloRange(env,1,expr,1));
            expr.clear();
        }
        
        for (int i=1; i<=2*n; i++)
        {
            for (int j=0; j<=2*n+1; j++)
            {
                expr += x[i][j];
            }
            model.add(IloRange(env,1,expr,1));
            expr.clear();
        }

        // additional constraints number of vehicles
        for (int j=1; j<=2*n+1; j++)
        {
            expr += x[0][j];
        }
        model.add(IloRange(env, expr, D.num_vehicles));
        expr.clear();
        
        // time windows
        for (int i=0; i<=2*n; i++)
        {
            model.add(IloRange(env,D.nodes[i].start_tw,B[i],D.nodes[i].end_tw));
        }
        model.add(IloRange(env,D.nodes[0].start_tw,B[2*n+1],D.nodes[0].end_tw));


        // i \in {0,2n}
        for (int i=0; i<=2*n; i++)
        {
            // j \in {0,2n}
            for (int j=0; j<=2*n; j++)
            {
                expr = -B[j] + B[i] + D.nodes[i].service_time + D.d[i][j] - DARPH_MAX(0,D.nodes[i].service_time + D.d[i][j] + D.nodes[i].end_tw - D.nodes[j].start_tw) * (1 - x[i][j]);
                model.add(IloRange(env,-IloInfinity,expr,0));
                expr.clear();
            }
            
            // j = 2n+1
            expr = -B[2*n+1] + B[i] + D.nodes[i].service_time + D.d[i][0] - DARPH_MAX(0,D.nodes[i].service_time + D.d[i][0] + D.nodes[i].end_tw - D.nodes[0].start_tw) * (1 - x[i][2*n+1]);
            model.add(IloRange(env,-IloInfinity,expr,0));
            expr.clear();     
        }        

        
        for (int i=1; i<=n; i++)
        {
            expr = -B[n+i] + B[i] + D.nodes[i].service_time + D.d[i][n+i];
            model.add(IloRange(env,-IloInfinity,expr,0));
            expr.clear();
        }

        // additional ride time constraints
        for (int i=1; i<=n; i++)
        {
            model.add(IloRange(env,B[n+i] - (B[i] + D.nodes[i].service_time), D.nodes[i].max_ride_time));
        }

        // additional constraints maximum duration of vehicle tours
        model.add(IloRange(env,-IloInfinity,B[2*n+1] - B[0], D.max_route_duration));


        // i \in {0,2n} 
        for (int i=0; i<=2*n; i++)
        {
            // j \in {0,2n}
            for (int j=0; j<=2*n; j++)
            {
                expr = -QQ[j] + QQ[i] + D.nodes[j].demand - min(D.veh_capacity, D.veh_capacity + D.nodes[i].demand) * (1 - x[i][j]);
                model.add(IloRange(env,-IloInfinity,expr,0));
                expr.clear();
            }
            
            // j = 2n+1
            expr = -QQ[2*n+1] + QQ[i] + D.nodes[0].demand - min(D.veh_capacity, D.veh_capacity + D.nodes[i].demand) * (1 - x[i][2*n+1]);
            model.add(IloRange(env,-IloInfinity,expr,0));
            expr.clear();
            
        }

        for (int i=1; i<=n; i++)
        {
            expr = v[n+i] - v[i];
            model.add(IloRange(env,0,expr,0));
            expr.clear();
        }
        
        for (int j=1; j<=2*n; j++)
        {
            expr = -v[j] + j * x[0][j];
            model.add(IloRange(env,-IloInfinity,expr,0));
            expr.clear();

            expr = -v[j] + j * x[0][j] - n * (x[0][j] - 1);
            model.add(IloRange(env,0,expr,IloInfinity));
            expr.clear();
        }

        for (int i=1; i<=2*n; i++)
        {
            for (int j=1; j<=2*n; j++)
            {
                expr = -v[j] + v[i] + n * (x[i][j] - 1);
                model.add(IloRange(env,-IloInfinity,expr,0));
                expr.clear();

                expr = -v[j] + v[i] + n * (1 - x[i][j]);
                model.add(IloRange(env,0,expr,IloInfinity));
                expr.clear();
            }
        }
        
        // Free the memory used by expr
        expr.end(); 
        
        // Create the solver object
        IloCplex cplex(model);
        cplex.exportModel("qkp.lp");
        
        const sec dur_model = clock::now() - before;

        cplex.setParam(IloCplex::Param::TimeLimit, 7200);
                
        solved = cplex.solve();
        const sec dur_solve = clock::now() - before;   
        
        if (solved)
        {
            // If CPLEX successfully solved the model, print the results
            std::cout << "\n\nCplex success!\n";
            std::cout << "\tStatus : " << cplex.getStatus() << "\n";
            std::cout << "\tObjective value: " << cplex.getObjValue() << "\n";
            std::cout << "\tTotal routing costs: " << cplex.getValue(obj1) << "\n";
            std::cout << "\t Relative MIP Gap: " << cplex.getMIPRelativeGap() << "\n";
            std::cout << "\tModel MILP " << dur_model.count() << "s" << std::endl;
            std::cout << "\tModel + Solve MILP " << dur_solve.count() << "s" << std::endl;

            obj1.end();
        }
        else
        {
            std::cerr << "\n\nCplex error!\n";
            std::cerr << "\tStatus: " << cplex.getStatus() << "\n";
            std::cerr << "\tSolver status: " << cplex.getCplexStatus() << "\n";  
        }
    }
    catch (IloException& ex) {
        cerr << "Error: " << ex << endl;
    }
    catch (...) {
        cerr << "Error" << endl;
    }
    
    env.end();
    
    return true;
}


template <int Q>
bool EBMILP<Q>::solve_cordeau(DARP& D, DARPGraph<Q>& G)
{
    // measure model and solve time
    const auto before = clock::now();

    bool solved = false;

    // We use this stringstream to create variable and constraint names
    std::stringstream name;

    // create graph
    check_paths(D); // DARPSolver, line 38
    G.create_graph(D,f); // DARPGraph, line 634
    create_maps(D, G); // DARPGraph, line 16
    
    IloEnv env;
    
    try
    { 
        IloModel model(env);
        
        std::cout << "n = " << n;
        
        // Variables
        typedef IloArray<IloNumVarArray> NumVarMatrix;
        typedef IloArray<NumVarMatrix>   NumVar3Matrix;
        NumVar3Matrix x(env,2*n+2); // nodes 0, 2n+1
        // initialize this matrix 
        for (int i = 0; i <= 2*n+1; i++) 
        {
            x[i] = NumVarMatrix(env, 2*n+2);
            for (int j = 0; j <= 2*n+1; j++)
            {
                x[i][j] = IloNumVarArray(env, D.num_vehicles);
                for (int k = 0; k < D.num_vehicles; k++)
                {
                    name << "x_{" << i << "," << j << "," << k << "}";
                    x[i][j][k] = IloNumVar(env, 0, 1, ILOBOOL, name.str().c_str());
                    name.str("");
                }
            }
        }

        // the time at which vehicle k begins service at node i
        IloNumVarArray B_0(env, D.num_vehicles);
        for (int k = 0; k < D.num_vehicles; k++)
        {
            name << "B_0^" << k;
            B_0[k] = IloNumVar(env, D.nodes[0].start_tw, D.nodes[0].end_tw, ILOFLOAT, name.str().c_str());
            name.str(""); // Clean name
        }
        
        IloNumVarArray B_2n1(env, D.num_vehicles);
        for (int k = 0; k < D.num_vehicles; k++)
        {
            name << "B_{2n+1}^" << k;
            B_2n1[k] = IloNumVar(env, D.nodes[0].start_tw, D.nodes[0].end_tw, ILOFLOAT, name.str().c_str());
            name.str(""); // Clean name
        }
        
        IloNumVarArray B(env,2*n+1);
        // B[0] weglassen weil es B_0^k gibt ~ Omit B[0] because there is B_0^k
        for (int i = 1; i <= 2*n; i++)
        {
            name << "B_" << i;
            B[i] = IloNumVar(env, D.nodes[i].start_tw, D.nodes[i].end_tw, ILOFLOAT, name.str().c_str());
            name.str("");
        }

        // ride time of user i
        IloNumVarArray L(env, n+1);
        // L[0] weglassen ~ Omit L[0]
        for (int i = 1; i <= n; i++)
        {
            name << "L_" << i;
            L[i] = IloNumVar(env, D.tt[i][n+i], D.nodes[i].max_ride_time, ILOFLOAT, name.str().c_str());
            name.str("");
        }
  
        // the load of vehicle k after visiting node i
        IloNumVarArray QQ_0(env, D.num_vehicles);
        for (int k = 0; k < D.num_vehicles; k++)
        {
            name << "Q_0^" << k;
            QQ_0[k] = IloNumVar(env, 0, D.veh_capacity, ILOFLOAT, name.str().c_str());
            name.str(""); // Clean name
        }
        IloNumVarArray QQ_2n1(env, D.num_vehicles);
        for (int k = 0; k < D.num_vehicles; k++)
        {
            name << "Q_{2n+1}^" << k;
            QQ_2n1[k] = IloNumVar(env, 0, D.veh_capacity, ILOFLOAT, name.str().c_str());
            name.str(""); // Clean name
        }
        IloNumVarArray QQ(env,2*n+1);
        // QQ[0] weglassen wegen Q_0^k ~ Omit QQ[0] because of Q_0^k
        for (int i = 1; i <= 2*n; i++)
        {
            name << "Q_" << i;
            QQ[i] = IloNumVar(env, max(0, D.nodes[i].demand), min(D.veh_capacity, D.veh_capacity + D.nodes[i].demand), ILOFLOAT, name.str().c_str());
            name.str("");
        }
    
        
        // Constraints
        IloRangeArray pick_up(env,n+1);
        IloArray<IloRangeArray> drop_off(env,D.num_vehicles);
        IloRangeArray leave_depot(env,D.num_vehicles);
        IloRangeArray return_depot(env,D.num_vehicles);
        IloArray<IloRangeArray> flow_preservation(env,D.num_vehicles);
   

        IloExpr obj1(env); // routing costs
        
        // Create objective function
        for (int i = 0; i <= 2*n; i++) {   
            // j \in {0, 2n}
            for (int j = 0; j <= 2*n; j++) {
                for (int k = 0; k < D.num_vehicles; k++) {
                    obj1 += D.d[i][j] * x[i][j][k];  
                }
            }
            // j = 2n+1   
            for (int k = 0; k < D.num_vehicles; k++) {
                obj1 += D.d[i][0] * x[i][2*n+1][k];
            } 
        }
        // i = 2n+1
        // j \in {0, 2n}
        for (int j = 0; j <= 2*n; j++) {
            for (int k = 0; k < D.num_vehicles; k++) {
                obj1 += D.d[0][j] * x[2*n+1][j][k];  
            }
        } 
       
        IloObjective obj(env, obj1, IloObjective::Minimize);
        model.add(obj);
        
        
        // Constraints
        IloExpr expr(env);
        
        
        for (int k = 0; k < D.num_vehicles; k++) {
            // x_{i,i} = 0 for all i
            for (int i = 0; i<=2*n+1; i++) {
                model.add(IloRange(env,0,x[i][i][k],0));
            }
            // x_{i,0} = 0 for all i \in P U D and i = 2*n+1
            for (int i = 1; i<=2*n+1; i++) {
                model.add(IloRange(env,0,x[i][0][k],0));
            }
            // x_{2*n+1,i} = 0 for all i \in P U D and i = 0
            for (int i = 0; i<=2*n; i++) {
                model.add(IloRange(env,0,x[2*n+1][i][k],0));
            }
            // x_{i,2*n+1} = 0 for all i \in P 
            // x_{n+i,i} = 0 for all i \in P 
            for (int i = 1; i<=n; i++) {
                model.add(IloRange(env,0,x[i][2*n+1][k],0));
                model.add(IloRange(env,0,x[n+i][i][k],0));
            }
            // x_{0,i} = 0 for all i \in D
            for (int i = n+1; i<=2*n; i++) {
                model.add(IloRange(env,0,x[0][i][k],0));
            }
        }
        
        for (int i=1; i<=n; i++) {
            for (int j=1; j<=n; j++) {
                if (j != i) {
                    // x_{i,j} = 0
                    if (f[j][i][0] == 0 && f[j][i][1] == 0) {
                        for (int k=0; k<D.num_vehicles; k++) {
                            model.add(IloRange(env,0,x[i][j][k],0));
                        }
                    }
                    // x_{n+i,n+j} = 0
                    if (f[j][i][0] == 0 && f[i][j][1] == 0) {
                        for (int k=0; k<D.num_vehicles; k++) {
                            model.add(IloRange(env,0,x[n+i][n+j][k],0));
                        }
                    }
                    // x_{i,n+j} = 0
                    if (f[i][j][0] == 0) {
                        for (int k=0; k<D.num_vehicles; k++) {
                            model.add(IloRange(env,0,x[i][n+j][k],0));
                        }
                    }
                } 
            }
        }
        
        // pick_up - jeder user wird nur von genau einem Fahrzeug abgeholt und fährt danach zu genau einem anderen Knoten
        // ~ pick_up - each user is picked up by exactly one vehicle and then travels to exactly one different node
        for (int i = 1; i<=n; i++) {
            for (int j = 0; j<=2*n+1; j++) {
                if (j != i) {
                    for (int k=0; k < D.num_vehicles; k++) {
                        expr += x[i][j][k];
                    }
                } 
            }
            name << "pick_up_" << i;
            pick_up[i] = IloRange(env, 1, expr, 1, name.str().c_str());
            name.str("");
            expr.clear();
        }
        model.add(pick_up);
        
        IloExpr expr2(env);
        // drop_off - jeder der abgeholt wird muss auch wieder abgesetzt werden (von demselben Fahrzeug)
        // ~ drop_off - everyone who is picked up must also be dropped off again (by the same vehicle)
        for (int k=0; k<D.num_vehicles; k++) {
            drop_off[k] = IloRangeArray(env,n+1);
            for (int i = 1; i<=n; i++) {
                for (int j = 0; j<=2*n+1; j++) {
                    if (j!=i)
                        expr += x[i][j][k];
                }
                for (int j = 0; j<=2*n+1; j++) {
                    if (j!=n+i)
                        expr2 += x[n+i][j][k];
                }
                name << "drop_off_{" << k << "," << i << "}";
                drop_off[k][i] = IloRange(env, 0, expr - expr2, 0, name.str().c_str());
                name.str("");
                expr.clear();
                expr2.clear();
            }
            model.add(drop_off[k]);
        }

        // leave_depot - jedes Fahrzeug verlässt das Depot
        // ~ leave_depot - every vehicle leaves the depot
        for (int k = 0; k<D.num_vehicles; k++) {
            for (int j = 1; j<=2*n+1; j++) {
                expr += x[0][j][k];
            }
            name << "leave_depot_" << k;
            leave_depot[k] = IloRange(env, 1, expr, 1, name.str().c_str());
            name.str("");
            expr.clear();
        }
        model.add(leave_depot);

        // flow preservation
        for (int k=0; k<D.num_vehicles; k++) {
            flow_preservation[k] = IloRangeArray(env,D.num_nodes+2);
            for (int i=1; i<=2*n; i++) {
                for (int j = 0; j<=2*n+1; j++) {
                    if (j!=i) expr += x[j][i][k] - x[i][j][k];
                }
                name << "flow_preservation_{" << k << "," << i << "}";
                flow_preservation[k][i] = IloRange(env, 0, expr, 0, name.str().c_str());
                name.str("");
                expr.clear();
            }
            model.add(flow_preservation[k]);
        }
        
        // return_depot
        for (int k=0; k<D.num_vehicles; k++) {
            for (int i=0; i<=2*n; i++) {
                expr += x[i][2*n+1][k];
            }
            name << "return_depot_" << k;
            return_depot[k] = IloRange(env, 1, expr, 1, name.str().c_str());
            name.str("");
            expr.clear();
        }
        model.add(return_depot);
        
        
        for (int k=0; k<D.num_vehicles; k++) {
            for (int j=1; j<=2*n; j++) {
                expr = -B[j] + B_0[k] + D.nodes[0].service_time + D.d[0][j] - DARPH_MAX(0, D.nodes[0].end_tw + D.nodes[0].service_time + D.d[0][j] - D.nodes[j].start_tw) * (1 - x[0][j][k]);
                model.add(IloRange(env,expr,0));
                expr.clear();
            }
        }

        for (int i=1; i<=2*n; i++) {
            for (int j=1; j<=2*n; j++) {
                if (j!=i) {
                    for (int k=0; k<D.num_vehicles; k++) {
                        expr2 += x[i][j][k];
                    }
                    expr = -B[j] + B[i] + D.nodes[i].service_time + D.d[i][j] - DARPH_MAX(0, D.nodes[i].end_tw + D.nodes[i].service_time + D.d[i][j] - D.nodes[j].start_tw) * (1 - expr2);
                    model.add(IloRange(env,expr,0));
                    expr.clear();
                    expr2.clear();
                }
            }
        }

        for (int k=0; k<D.num_vehicles; k++) {
            for (int i=1; i<=2*n; i++) {
                expr = -B_2n1[k] + B[i] + D.nodes[i].service_time + D.d[i][0] - DARPH_MAX(0, D.nodes[i].end_tw + D.nodes[i].service_time + D.d[i][0] - D.nodes[0].start_tw) * (1 - x[i][2*n+1][k]);
                model.add(IloRange(env,expr,0));
                expr.clear();
            }
        }
        
        for (int k=0; k<D.num_vehicles; k++) {
            expr = -B_2n1[k] + B_0[k] + D.nodes[0].service_time + D.d[0][0] - DARPH_MAX(0, D.nodes[0].end_tw + D.nodes[0].service_time + D.d[0][0] - D.nodes[0].start_tw) * (1-x[0][2*n+1][k]);
            model.add(IloRange(env,expr,0));
            expr.clear();
        }
        
        // ride time
        for (int i=1; i<=n; i++) {
            expr = L[i] - B[n+i] + (B[i] + D.nodes[i].service_time);
            name << "ride_time_" << i;
            model.add(IloRange(env, 0, expr, 0, name.str().c_str()));
            name.str("");
            expr.clear();
        }
        
        // Capacity
        for (int k=0; k<D.num_vehicles; k++)
        {
            for (int j=1; j<=2*n; j++)
            {
                expr = -QQ[j] + QQ_0[k] + D.nodes[j].demand - min(D.veh_capacity, D.veh_capacity + D.nodes[0].demand) * (1-x[0][j][k]);
                model.add(IloRange(env,expr,0));
                expr.clear();
            }
        }
        
        IloExpr expr3(env);
        for (int i=1; i<=2*n; i++)
        {
            for (int j=1; j<=2*n; j++)
            {
                if (j!=i)
                {
                    for (int k=0; k<D.num_vehicles; k++)
                    {
                        expr2 += x[i][j][k];
                        expr3 += x[j][i][k];
                    }

                    expr = -QQ[j] + QQ[i] + D.nodes[j].demand - min(D.veh_capacity, D.veh_capacity + D.nodes[i].demand) * (1 - expr2) + (min(D.veh_capacity, D.veh_capacity + D.nodes[i].demand) - D.nodes[i].demand - D.nodes[j].demand) * expr3;
                    model.add(IloRange(env,expr,0));
                    expr.clear();
                    expr2.clear();
                    expr3.clear();
                    
                }
            }
        }
        
        for (int k=0; k<D.num_vehicles; k++)
        {
            for (int i=1; i<=2*n; i++)
            {
                expr = -QQ_2n1[k] + QQ[i] + D.nodes[0].demand - min(D.veh_capacity, D.veh_capacity + D.nodes[i].demand) * (1 - x[i][2*n+1][k]);
                model.add(IloRange(env,expr,0));
                expr.clear();
            }
        }
        
        for (int k=0; k<D.num_vehicles; k++)
        {
            expr = -QQ_2n1[k] + QQ_0[k] + D.nodes[0].demand - min(D.veh_capacity, D.veh_capacity + D.nodes[0].demand) * (1 - x[0][2*n+1][k]);
            model.add(IloRange(env,expr,0));
            expr.clear();
        }
        
        // maximum duration of vehicle tour
        for (int k=0; k<D.num_vehicles; k++)
        {
            model.add(IloRange(env, B_2n1[k] - B_0[k],D.max_route_duration));
        }
        
        
        // Free the memory used by expr
        expr.end(); 
        expr2.end();
        expr3.end();

        
        // Create the solver object
        IloCplex cplex(model);
        cplex.exportModel("dar_problem.lp");
        
        const sec dur_model = clock::now() - before;

        
        cplex.setParam(IloCplex::Param::Simplex::Tolerances::Feasibility, 0.0001);

        cplex.setParam(IloCplex::Param::TimeLimit, 7200);
        solved = cplex.solve();
        const sec dur_solve = clock::now() - before;

        
        if (solved)
        {
            // If CPLEX successfully solved the model, print the results
            std::cout << "\n\nCplex success!\n";
            std::cout << "\tStatus tt: " << cplex.getStatus() << "\n";
            std::cout << "\tObjective value: " << cplex.getObjValue() << "\n";
            std::cout << "\tTotal routing costs: " << cplex.getValue(obj1) << "\n";
            std::cout << "\t Relative MIP Gap: " << cplex.getMIPRelativeGap() << "\n";
            std::cout << "\tModel MILP " << dur_model.count() << "s" << std::endl;
            std::cout << "\tModel + Solve MILP " << dur_solve.count() << "s" << std::endl;

            

            obj1.end();
    
        }
        else
        {
            std::cerr << "\n\nCplex error!\n";
            std::cerr << "\tStatus: " << cplex.getStatus() << "\n";
            std::cerr << "\tSolver status: " << cplex.getCplexStatus() << "\n";
            
        }        
    }
    catch (IloException& ex) {
        cerr << "Error: " << ex << endl;
    }
    catch (...) {
        cerr << "Error" << endl;
    }

    env.end();
     
    return true;
}

template <int Q>
bool EBMILP<Q>::solve_milpI(DARP& D, DARPGraph<Q>& G)
{
    // measure model and solve time
    const auto before = clock::now();

    bool solved = false;

    // We use this stringstream to create variable and constraint names
    std::stringstream name;

    // create graph
    check_paths(D);
    G.create_graph(D,f); 
    create_maps(D, G);

    IloEnv env;
    try
    { 
        IloModel model(env);
        
        // Variables
        // array for variables B_v
        IloNumVarArray B(env, G.vcardinality);
        if constexpr (Q==3)
            B[vmap[G.depot]] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, "B_(0,0,0)");
        else
            B[vmap[G.depot]] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, "B_(0,0,0,0,0,0)");
        for (const auto& v: G.V_in)
        {
            if constexpr (Q==3)
                name << "B_(" << v[0] << "," << v[1] << "," << v[2] << ")";
            else
                name << "B_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
            B[vmap[v]] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, name.str().c_str());
            name.str(""); // Clean name
        }
        for (const auto& v: G.V_out)
        {
            if constexpr (Q==3)
                name << "B_(" << v[0] << "," << v[1] << "," << v[2] << ")";
            else
                name << "B_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
            B[vmap[v]] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, name.str().c_str());
            name.str(""); // Clean name
        }
        
        // array for variables x_a
        IloNumVarArray x(env, G.acardinality);
        for (const auto& a: G.A)
        {
            if constexpr (Q==3)
                name << "x_(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
            else
                name << "x_(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
            x[amap[a]] = IloNumVar(env, 0, 1, ILOBOOL, name.str().c_str());
            name.str(""); // Clean name
        }
        

        
        // Constraints
        IloRangeArray flow_preservation(env,G.vcardinality);
        IloRangeArray serve_accepted(env,D.num_requests);
        IloRangeArray travel_time(env,G.acardinality);
        IloRangeArray time_window(env,G.vcardinality);
        IloArray<IloRangeArray> max_ride_time(env,G.vincardinality);
        
        
        IloExpr obj1(env); // routing costs
        IloExpr expr(env);
        
        // Create objective function
        for (const auto& a: G.A)
        {   
            obj1 += G.c[a] * x[amap[a]];   
        }
        IloObjective obj(env, obj1, IloObjective::Minimize);
        model.add(obj); 

        // Constraints
        // 'flow preservation' constraints
        // depot
        for (const auto& a: G.delta_in[G.depot])
        {
            expr += x[amap[a]];
        }
        for(const auto& a: G.delta_out[G.depot])
        {
            expr -= x[amap[a]];
        }
        if constexpr (Q==3)
            flow_preservation[vmap[G.depot]] = IloRange(env,0,expr,0,"flow_preservation_(0,0,0)");
        else
            flow_preservation[vmap[G.depot]] = IloRange(env,0,expr,0,"flow_preservation_(0,0,0,0,0,0)");
        expr.clear();
        // V_in
        for (const auto& v: G.V_in)
        {
            if constexpr (Q==3)
                name << "flow_preservation_B_(" << v[0] << "," << v[1] << "," << v[2] << ")";
            else
                name << "flow_preservation_B_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";

            for(const auto& a: G.delta_in[v])
            {
                expr += x[amap[a]];
            }
            for(const auto& a: G.delta_out[v])
            {
                expr -= x[amap[a]];
            }
            flow_preservation[vmap[v]] = IloRange(env,0,expr,0,name.str().c_str());
            expr.clear();
            name.str("");
        }
        // V_out
        for (const auto& v: G.V_out)
        {
            if constexpr (Q==3)
                name << "flow_preservation_B_(" << v[0] << "," << v[1] << "," << v[2] << ")";
            else
                name << "flow_preservation_B_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
            for(const auto& a: G.delta_in[v])
            {
                expr += x[amap[a]];
            }
            for(const auto& a: G.delta_out[v])
            {
                expr -= x[amap[a]];
            }
            flow_preservation[vmap[v]] = IloRange(env,0,expr,0,name.str().c_str());
            expr.clear();
            name.str("");
        }
        model.add(flow_preservation);
       
        
        
        // every request accepted has to be served
        for (const auto& i : D.R)
        {
            name << "serve_accepted_" << i;
            for (const auto& v: G.V_i[i])
            {
                for (const auto& a: G.delta_in[v])
                {
                    expr += x[amap[a]];
                }
            }
            serve_accepted[rmap[i]] = IloRange(env,1,expr,1,name.str().c_str()); 
            expr.clear();
            name.str("");
        }
        model.add(serve_accepted);
        
        
        // number of vehicles
        for (const auto& a: G.delta_out[G.depot])
        {
            expr += x[amap[a]];
        }
        IloRange num_tours(env, expr, D.num_vehicles, "number_tours");
        model.add(num_tours);
        expr.clear();
        
        
        // travel time arc a 
        for (const auto& a: G.A)
        {   
            if constexpr (Q==3)
                name << "travel_time_(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "),(" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
            else
                name << "travel_time_(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "),(" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
            if (a[0] != G.depot)
            {
                expr = -B[vmap[a[1]]] + B[vmap[a[0]]] + D.nodes[a[0][0]].service_time + G.t[a] - (D.nodes[a[0][0]].end_tw - D.nodes[a[1][0]].start_tw + G.t[a] + D.nodes[a[0][0]].service_time) * (1 - x[amap[a]]);
                travel_time[amap[a]] = IloRange(env,expr,0,name.str().c_str());
            }
            else
            {
                expr = -B[vmap[a[1]]] + G.t[a] * x[amap[a]];
                travel_time[amap[a]] = IloRange(env,expr,0,name.str().c_str());
            }
            expr.clear();
            name.str("");
        }
        model.add(travel_time);
        

        
        // time constraints pick-up and drop-off
        
        // return to depot of last vehicle
        if constexpr (Q==3)
            time_window[vmap[G.depot]] = IloRange(env,B[vmap[G.depot]],D.max_route_duration,"time_window_ub_(0,0,0)");
        else
            time_window[vmap[G.depot]] = IloRange(env,B[vmap[G.depot]],D.max_route_duration,"time_window_ub_(0,0,0,0,0,0)");

        // inbound: tw_length = l_i - e_i 
        // outbound: tw_length = l_n+i - e_n+i 
        for (const auto& i : D.R)
        {
            for (const auto& v: G.V_i[i])
            {
                if constexpr (Q==3)
                    name << "time_window_(" << v[0] << "," << v[1] << "," << v[2] << ")";
                else
                    name << "time_window_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
                time_window[vmap[v]] = IloRange(env,D.nodes[i].start_tw,B[vmap[v]],D.nodes[i].end_tw,name.str().c_str()); 
                name.str("");
            }
            for (const auto& v: G.V_i[n+i])
            {
                if constexpr (Q==3)
                    name << "time_window_(" << v[0] << "," << v[1] << "," << v[2] << ")";
                else
                    name << "time_window_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
                time_window[vmap[v]] = IloRange(env, D.nodes[n+i].start_tw, B[vmap[v]], D.nodes[n+i].end_tw, name.str().c_str());
                name.str("");
            }      
        }
        model.add(time_window);
     
        
        IloExpr expr1(env);
        IloExpr expr2(env);
        
        // maximum ride time
        for (const auto& i: D.R)
        {
            for (const auto& v: G.V_i[i])
            {
                max_ride_time[vinmap[v]] = IloRangeArray(env,G.V_i[n+i].size());
                for (const auto& a: G.delta_in[v])
                {
                    expr1 += x[amap[a]];
                }
                for (const auto& w: G.V_i[n+i])
                {
                    if constexpr (Q==3)
                        name << "max_ride_time_(" << v[0] << "," << v[1] << "," << v[2] << ") -- ("<< w[0] << "," << w[1] << "," << w[2] << ")";
                    else
                        name << "max_ride_time_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ") -- ("<< w[0] << "," << w[1] << "," << w[2] << "," << w[3] << "," << w[4] << "," << w[5] << ")";
                    for (const auto& a: G.delta_in[w])
                    {
                        expr2 += x[amap[a]];
                    }
                    expr = B[vmap[w]] - B[vmap[v]] - D.nodes[i].service_time - (D.nodes[n+i].end_tw - D.nodes[i].start_tw - D.nodes[i].max_ride_time - D.nodes[i].service_time) * (1-expr1 + 1-expr2);
                    max_ride_time[vinmap[v]][vec_map[i-1][w]] = IloRange(env,expr,D.nodes[i].max_ride_time,name.str().c_str());
                    expr.clear();
                    expr2.clear();
                    name.str("");
                }
                model.add(max_ride_time[vinmap[v]]);
                expr1.clear();
            }
        }
        
        
        // Free the memory used by expr
        expr.end(); 
        obj1.end();
        expr1.end();
        expr2.end();
       
        // Create the solver object
        IloCplex cplex(model);
        //cplex.exportModel("MILP1.lp");
        
        const sec dur_model = clock::now() - before;
                
        cplex.setParam(IloCplex::Param::Simplex::Tolerances::Feasibility, 0.0001);
       
        cplex.setParam(IloCplex::Param::TimeLimit, 7200);
        solved = cplex.solve();
        const sec dur_solve = clock::now() - before;
 
        if (solved)
        {
            // If CPLEX successfully solved the model, print the results
            std::cout << "\n\nCplex success!\n";
            std::cout << "\tStatus: " << cplex.getStatus() << "\n";
            std::cout << "\tObjective value: " << cplex.getObjValue() << "\n";
            std::cout << "\t Relative MIP Gap: " << cplex.getMIPRelativeGap() << "\n";
            std::cout << "\tModel MILP " << dur_model.count() << "s" << std::endl;
            std::cout << "\tModel + Solve MILP " << dur_solve.count() << "s" << std::endl;                 
        }
        else
        {
            std::cerr << "\n\nCplex error!\n";
            std::cerr << "\tStatus: " << cplex.getStatus() << "\n";
            std::cerr << "\tSolver status: " << cplex.getCplexStatus() << "\n";
        }
        
    }
    catch (IloException& ex) {
        cerr << "Error: " << ex << endl;
    }
    catch (...) {
        cerr << "Error" << endl;
    }

    env.end();
     
    return true;
}



template <int Q>
bool EBMILP<Q>::solve(bool accept_all, bool consider_excess_ride_time, DARP& D, DARPGraph<Q>& G, double w1, double w2, double w3)
{
    // We use this stringstream to create variable and constraint names
    std::stringstream name;

    bool solved;

    // create Graph
    check_paths(D);
    G.create_graph(D,f);
    create_maps(D, G);

    try
    { 
        const auto before = clock::now();
        
        model = IloModel(env);

        
        // Variables
        // array for variables B_v
        B = IloNumVarArray(env, G.vcardinality);
        if constexpr (Q==3)
            B[vmap[G.depot]] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, "B_(0,0,0)");
        else
            B[vmap[G.depot]] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, "B_(0,0,0,0,0,0)");
        for (const auto& v: G.V_in)
        {
            if constexpr (Q==3)
                name << "B_(" << v[0] << "," << v[1] << "," << v[2] << ")";
            else
                name << "B_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
            B[vmap[v]] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, name.str().c_str());
            name.str(""); // Clean name
        }
        for (const auto& v: G.V_out)
        {
            if constexpr (Q==3)
                name << "B_(" << v[0] << "," << v[1] << "," << v[2] << ")";
            else
                name << "B_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
            B[vmap[v]] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, name.str().c_str());
            name.str(""); // Clean name
        }
        // array for variables x_a
        x = IloNumVarArray(env, G.acardinality);
        for (const auto& a: G.A)
        {
            if constexpr (Q==3)
                name << "x_(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
            else
                name << "x_(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
            x[amap[a]] = IloNumVar(env, 0, 1, ILOBOOL, name.str().c_str());
            name.str(""); // Clean name
        }
        // array for variables p_i
        p = IloNumVarArray(env, D.num_requests);
        for (const auto& i: D.R)
        {
            name << "p_" << i;
            p[rmap[i]] = IloNumVar(env, 0, 1, ILOBOOL, name.str().c_str()); 
            name.str(""); // Clean name
        }
        // array for variables d_i
        d = IloNumVarArray(env, D.num_requests);
        for (const auto& i: D.R)
        {
            name << "d_" << i;
            d[rmap[i]] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, name.str().c_str());
            name.str(""); // Clean name
        }
        d_max = IloNumVar(env,0,IloInfinity, ILOFLOAT, name.str().c_str());

        
        // to save solution values
        B_val = IloNumArray(env, G.vcardinality);
        p_val = IloIntArray(env, D.num_requests);
        x_val = IloIntArray(env, G.acardinality);
        d_val = IloNumArray(env, D.num_requests);
        
        
        
        // Constraints
        accept = IloRangeArray(env, D.num_requests);
        flow_preservation = IloRangeArray(env, G.vcardinality);
        serve_accepted = IloRangeArray(env, D.num_requests);
        travel_time = IloRangeArray(env, G.acardinality);
        time_window_ub = IloRangeArray(env, G.vcardinality);
        time_window_lb = IloRangeArray(env, G.vcardinality);
        max_ride_time = IloArray<IloRangeArray>(env, G.vincardinality);
        fixed_B = IloRangeArray(env, G.vcardinality);
        fixed_x = IloRangeArray(env, G.acardinality);
        excess_ride_time = IloRangeArray(env, G.voutcardinality);
        pickup_delay = IloRangeArray(env, G.vincardinality);
        
        
        
        if (accept_all)
        {
            for (const auto& i : D.R)
            {
                name << "accept_" << i;
                accept[rmap[i]] = IloRange(env,1,p[rmap[i]],1,name.str().c_str()); 
                name.str(""); // Clean name
            }   
            model.add(accept);
        }
            
        obj1 = IloExpr(env); // routing costs
        obj2 = IloExpr(env); // answered requests
        obj3 = IloExpr(env); // excess ride time
        
        
        IloExpr expr(env);

        // Create objective function
        obj1 += 0;
        for (const auto& a: G.A)
        {   
            obj1 += G.c[a] * x[amap[a]]; 
        }
        
        obj2 += 0;
        if (!accept_all)
        {
            obj2 += D.num_requests; 
            for (const auto& i : D.R)
            {
                obj2 -= p[rmap[i]]; 
            }
        }
        
        obj3 += 0;
        if (consider_excess_ride_time)
        {
            for (const auto& i: D.R)
            {
                obj3 += d[rmap[i]];
            }
        }
        
        expr = w1 * obj1 + w2 * obj2 + w3 * obj3; 
        obj = IloObjective(env, expr, IloObjective::Minimize);
        model.add(obj);
        expr.clear();
        
        // Constraints
        // 'flow preservation' constraints
        // depot
        for (const auto& a: G.delta_in[G.depot])
        {
            expr += x[amap[a]];
        }
        for(const auto& a: G.delta_out[G.depot])
        {
            expr -= x[amap[a]];
        }
        if constexpr (Q==3)
            flow_preservation[vmap[G.depot]] = IloRange(env,0,expr,0,"flow_preservation_(0,0,0)");
        else
            flow_preservation[vmap[G.depot]] = IloRange(env,0,expr,0,"flow_preservation_(0,0,0,0,0,0)");
        expr.clear();
        // V_in
        for (const auto& v: G.V_in)
        {
            if constexpr (Q==3)
                name << "flow_preservation_B_(" << v[0] << "," << v[1] << "," << v[2] << ")";
            else
                name << "flow_preservation_B_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";

            for(const auto& a: G.delta_in[v])
            {
                expr += x[amap[a]];
            }
            for(const auto& a: G.delta_out[v])
            {
                expr -= x[amap[a]];
            }
            flow_preservation[vmap[v]] = IloRange(env,0,expr,0,name.str().c_str());
            expr.clear();
            name.str("");
        }
        // V_out
        for (const auto& v: G.V_out)
        {
            if constexpr (Q==3)
                name << "flow_preservation_B_(" << v[0] << "," << v[1] << "," << v[2] << ")";
            else
                name << "flow_preservation_B_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
            for(const auto& a: G.delta_in[v])
            {
                expr += x[amap[a]];
            }
            for(const auto& a: G.delta_out[v])
            {
                expr -= x[amap[a]];
            }
            flow_preservation[vmap[v]] = IloRange(env,0,expr,0,name.str().c_str());
            expr.clear();
            name.str("");
        }
        model.add(flow_preservation);
        
        
        
        // every request accepted has to be served
        for (const auto& i : D.R)
        {
            name << "serve_accepted_" << i;
            for (const auto& v: G.V_i[i])
            {
                for (const auto& a: G.delta_in[v])
                {
                    expr += x[amap[a]];
                }
            }
            expr -= p[rmap[i]];
            serve_accepted[rmap[i]] = IloRange(env,0,expr,0,name.str().c_str()); 
            expr.clear();
            name.str("");
        }
        model.add(serve_accepted);
        

        // number of vehicles
        for (const auto& a: G.delta_out[G.depot])
        {
            expr += x[amap[a]];
        }
        num_tours = IloRange(env, expr, D.num_vehicles, "number_tours");
        model.add(num_tours);
        expr.clear();
        

        // travel time arc a 
        for (const auto& a: G.A)
        {   
            if constexpr (Q==3)
                name << "travel_time_(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "),(" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
            else
                name << "travel_time_(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "),(" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
            if (a[0] != G.depot)
            {
                expr = -B[vmap[a[1]]] + B[vmap[a[0]]] + D.nodes[a[0][0]].service_time + G.t[a] - (D.nodes[a[0][0]].end_tw - D.nodes[a[1][0]].start_tw + G.t[a] + D.nodes[a[0][0]].service_time) * (1 - x[amap[a]]);
                travel_time[amap[a]] = IloRange(env,expr,0,name.str().c_str());
            }
            else
            {
                expr = -B[vmap[a[1]]] + G.t[a] * x[amap[a]];
                travel_time[amap[a]] = IloRange(env,expr,-D.nodes[0].start_tw,name.str().c_str());
            }
            expr.clear();
            name.str("");
        }
        model.add(travel_time);


        
        // time constraints pick-up and drop-off
        
        // return to depot of last vehicle
        if constexpr (Q==3)
            time_window_ub[vmap[G.depot]] = IloRange(env,B[vmap[G.depot]],D.max_route_duration,"time_window_ub_(0,0,0)");
        else
            time_window_ub[vmap[G.depot]] = IloRange(env,B[vmap[G.depot]],D.max_route_duration,"time_window_ub_(0,0,0,0,0,0)");

        // inbound: tw_length = l_i - e_i 
        // outbound: tw_length = l_n+i - e_n+i 
        for (const auto& i : D.R)
        {
            for (const auto& v: G.V_i[i])
            {
                for (const auto& a: G.delta_in[v])
                {
                    expr += x[amap[a]];
                }
                if constexpr (Q==3)
                    name << "time_window_lb_(" << v[0] << "," << v[1] << "," << v[2] << ")";
                else
                    name << "time_window_lb_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
                time_window_lb[vmap[v]] = IloRange(env,-B[vmap[v]] + D.nodes[i].start_tw + D.nodes[i].tw_length * (1 - expr),0,name.str().c_str()); 
                expr.clear();
                name.str("");

                if constexpr (Q==3)
                    name << "time_window_ub_(" << v[0] << "," << v[1] << "," << v[2] << ")";
                else
                    name << "time_window_ub_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
                time_window_ub[vmap[v]] = IloRange(env,B[vmap[v]],D.nodes[i].end_tw,name.str().c_str());
                name.str("");
            }
            for (const auto& v: G.V_i[n+i])
            {
                for (const auto& a: G.delta_in[v])
                {
                    expr += x[amap[a]];
                }
                if constexpr (Q==3)
                    name << "time_window_ub_(" << v[0] << "," << v[1] << "," << v[2] << ")";
                else
                    name << "time_window_ub_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
                time_window_ub[vmap[v]] = IloRange(env, 0, -B[vmap[v]] + D.nodes[i].max_ride_time + D.nodes[i].start_tw + D.nodes[i].service_time + D.nodes[i].tw_length * expr, IloInfinity, name.str().c_str());
                expr.clear();
                name.str("");

                if constexpr (Q==3)
                    name << "time_window_lb_(" << v[0] << "," << v[1] << "," << v[2] << ")";
                else
                    name << "time_window_lb_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
                time_window_lb[vmap[v]] = IloRange(env,D.nodes[n+i].start_tw,B[vmap[v]], IloInfinity, name.str().c_str());
                name.str("");
            }      
        }
        model.add(time_window_ub);
        model.add(time_window_lb);
        
        
        
        // maximum ride time
        for (const auto& i: D.R)
        {
            for (const auto& v: G.V_i[i])
            {
                max_ride_time[vinmap[v]] = IloRangeArray(env,G.V_i[n+i].size());
                for (const auto& w: G.V_i[n+i])
                {
                    if constexpr (Q==3)
                        name << "max_ride_time_(" << v[0] << "," << v[1] << "," << v[2] << ") -- ("<< w[0] << "," << w[1] << "," << w[2] << ")";
                    else
                        name << "max_ride_time_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ") -- ("<< w[0] << "," << w[1] << "," << w[2] << "," << w[3] << "," << w[4] << "," << w[5] << ")";
                    max_ride_time[vinmap[v]][vec_map[i-1][w]] = IloRange(env,B[vmap[w]] - (B[vmap[v]] + D.nodes[i].service_time),D.nodes[i].max_ride_time,name.str().c_str());
                    name.str("");
                }
                model.add(max_ride_time[vinmap[v]]);
            }
        }
        
        // excess ride time constraints
        if (consider_excess_ride_time)
        {
            for (const auto& v: G.V_out)
            {
                if constexpr (Q==3)
                    name << "excess_ride_time_(" << v[0] << "," << v[1] << "," << v[2] << ")";
                else
                    name << "excess_ride_time_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
                excess_ride_time[voutmap[v]] = IloRange(env, -IloInfinity, B[vmap[v]]-d[rmap[v[0]-n]], D.nodes[v[0]].start_tw,name.str().c_str()); 
                name.str("");
            }
            model.add(excess_ride_time);

            /*
            for (const auto& i: R)
            {
                model.add(IloRange(env,0,d_max-d[rmap[i]]));
            }
            */   
        }

        // Free the memory used by expr
        expr.end(); 
           
        // Create the solver object
        cplex = IloCplex(model);
        //name << "MILP/MILP" << "_w1=" << w1 << "_w2=" << w2 << "_w_3=" << w3 << ".lp";
        //cplex.exportModel(name.str().c_str());
        //name.str("");

        dur_model = clock::now() - before;
#if VERBOSE == 0
        cplex.setOut(env.getNullStream());
#endif
        cplex.setParam(IloCplex::Param::Simplex::Tolerances::Feasibility, 0.0001);
        
        cplex.setParam(IloCplex::Param::TimeLimit, 7200);

        solved = cplex.solve();
        dur_solve = clock::now() - before;

        if (solved)
        {
            // If CPLEX successfully solved the model, print the results
            
            
            get_solution_values(consider_excess_ride_time, D, G);
            
            total_routing_costs = cplex.getValue(obj1);
            std::cout << "Total routing costs: " << total_routing_costs << std::endl; 
#if VERBOSE
            std::cout << "Average routing costs: " << roundf(total_routing_costs / (n - all_denied.size()) * 100) / 100 << std::endl; 
#endif
            
            
            if (consider_excess_ride_time)
            {
                if (w3 > 0)
                {
                    // falls w3 = 0 wird nicht nach d_i optimiert, dementsprechend ist d_i evtl größer als B_i - e_{i⁻} 
                    total_excess_ride_time = cplex.getValue(obj3);
                    std::cout << "Total excess ride time: " << total_excess_ride_time << std::endl; 
#if VERBOSE
                    std::cout << "Average excess ride time: " << roundf(total_excess_ride_time / (n - all_denied.size()) * 100) / 100 << std::endl; 
#endif
                }
                else
                {
                    total_excess_ride_time = 0;
                    for (const auto& i: D.R)
                    {
                        if (std::find(all_denied.begin(), all_denied.end(), i) == all_denied.end())
                        {
                            total_excess_ride_time += D.nodes[n+i].beginning_service - D.nodes[n+i].start_tw;
                        }
                    }
                    std::cout << "Total excess ride time: " << total_excess_ride_time << std::endl; 
#if VERBOSE
                    std::cout << "Average excess ride time: " << roundf(total_excess_ride_time / (n - all_denied.size()) * 100) / 100 << std::endl; 
#endif
                }
            }
            
            answered_requests = n - cplex.getValue(obj2);
            

        }
        else
        {
            std::cerr << "\n\nCplex error!\n";
            std::cerr << "\tStatus: " << cplex.getStatus() << "\n";
            std::cerr << "\tSolver status: " << cplex.getCplexStatus() << "\n";
        }
        // free memory of Cplex objects created in first_milp()
        obj1.end();
        obj2.end();
        obj3.end();
        env.end(); 
    }
    catch (IloException& ex) {
        cerr << "Error: " << ex << endl;
    }
    catch (...) {
        cerr << "Error" << endl;
    }

    return true;
}


template class EBMILP<3>;
template class EBMILP<6>;
