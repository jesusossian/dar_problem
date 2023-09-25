#include <ilcplex/ilocplex.h>
#include <ilconcert/iloexpression.h>
ILOSTLBEGIN

#ifndef _DARPCPLEX_H
#define _DARPCPLEX_H


class DARPCplex {

public: 
    // Cplex model
    IloCplex cplex;
    IloEnv env;
    IloModel model;

    // save solution values
    IloNumArray B_val, d_val;
    IloIntArray p_val, x_val;  

    // Variables
    IloNumVarArray B, x, p, d;
    IloNumVar d_max;
        
    // Constraints
    IloRangeArray accept, serve_accepted;
    IloRangeArray time_window_ub, time_window_lb;
    IloArray<IloRangeArray> max_ride_time;
    IloRangeArray travel_time;
    IloRangeArray flow_preservation;
    IloRangeArray excess_ride_time;
    IloRangeArray fixed_B, fixed_x;
    IloRangeArray pickup_delay;
    IloRange num_tours;
        
    // Objective function
    IloObjective obj;       
    IloExpr obj1; // routing costs
    IloExpr obj2; // rejected requests
    IloExpr obj3; // excess ride time
};

#endif