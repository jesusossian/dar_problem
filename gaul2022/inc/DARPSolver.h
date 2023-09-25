#ifndef _DARP_SOLVER_H
#define _DARP_SOLVER_H


class DARPSolver {
public:
    int n; // num_requests
   
    // measuring model and solve time
    using clock = std::chrono::system_clock;
    using sec = std::chrono::duration<double>;
    
    // save requests' status
    std::vector<int> all_denied;

    // general evaluation
    double total_routing_costs;
    double total_excess_ride_time;
    double answered_requests;

    // check feasibility of paths
    int ***f; // path feasibilty matrix: 1 = feasible, 0 = infeasible   
    
public:
    DARPSolver(int);
    ~DARPSolver();

    // check pairwise feasibility of paths
    void check_paths(DARP& D);

    // 8-step route evaluation scheme by Cordeau and Laporte (2003) (modified)
    bool update_vertices(DARP& D, DARPRoute&); 
    bool update_vertices(DARP& D, DARPRoute&, int); 
    bool eight_step(DARP& D, DARPRoute&); 
    
};
    
#endif