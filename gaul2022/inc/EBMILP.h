#include <ilcplex/ilocplex.h>
#include <ilconcert/iloexpression.h>
ILOSTLBEGIN

#ifndef _EBMILP_H
#define _EBMILP_H

// Solve DARP using event-based MILP
template<int S>
class EBMILP : public DARPSolver, DARPCplex {
    
    typedef std::array<int,S> NODE;
    typedef std::array<std::array<int,S>,2> ARC;
public:
    
    // map requests to variables
    std::unordered_map <int,int> rmap;
    std::unordered_map <NODE,uint64_t,HashFunction<S>> vmap; // max. Anzahl Knoten: O(n^Q) --> passt in 64 bit integer
    std::unordered_map <NODE,uint64_t,HashFunction<S>> vinmap;
    std::unordered_map <NODE,uint64_t,HashFunction<S>> voutmap;
    std::unordered_map <ARC,uint64_t,HashFunction<S>> amap; // max. Anzahl Kanten: O(n^Q+1) --> passt in 64 bit integer
    std::unordered_map<NODE,int,HashFunction<S>>* vec_map; 

    // solve time
    double total_time_model;
    double total_time_model_solve;
    sec dur_model;
    sec dur_solve;

    
public:
    // Constructor
    EBMILP(int);
    ~EBMILP();

    // maps - nachdem nodes und arcs erstellt wurden!
    
    void create_maps(DARP& D, DARPGraph<S>& G);

    // solve different MILPs
    bool solve_milpI(DARP& D, DARPGraph<S>& G);
    bool solve_cordeau(DARP& D, DARPGraph<S>& G);
    bool solve_furtado(DARP& D, DARPGraph<S>& G);
    bool solve(bool accept_all, bool consider_excess_ride_time, DARP& D, DARPGraph<S>& G, double w1 = 1, double w2 = 60, double w3 = 0.1);
    // objective function weights
    // w1 = 1 weight routing costs
    // w2 = 60 weight rejected requests 
    // w3 = 0.1 weight excess ride time 

    //after solve
    void get_solution_values(bool consider_excess_ride_time, DARP& D, DARPGraph<S>& G);
    void print_routes(DARP& D, DARPGraph<S>& G);

};

#endif