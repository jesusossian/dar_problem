#include "DARPH.h"


DARPSolver::DARPSolver(int num_requests) : n{num_requests} {
    // Allocate memory for feasibility matrix
    f = new int**[n+1];
    for (int i = 0; i < n+1; i++)
    {
        // Allocate memory blocks for rows of each 2D array
        f[i] = new int*[n+1];
        for (int j = 0; j < n+1; j++) 
        {
            // Allocate memory blocks for columns of each 2D array
            f[i][j] = new int[2];
            f[i][j][0] = -DARPH_INFINITY;
            f[i][j][1] = -DARPH_INFINITY;
        }
    }
}


DARPSolver::~DARPSolver() {

     // deallocate memory f
    for (int i=0; i<n; ++i)
    {
        for (int j=0; j<n; ++j)
        {
            delete[] f[i][j];
        }
        delete[] f[i];
    }
    delete[] f;
}



void DARPSolver::check_paths(DARP& D)
{
    ///
    /// check paths 
    ///
    DARPRoute path;
    
    // first check if problem instance is infeasible due to e_0 + t_0i > l_i
    for (const auto& i: D.R)
    {
        if (D.nodes[DARPH_DEPOT].start_tw + D.tt[DARPH_DEPOT][i] > D.nodes[i].end_tw)
        {
            fprintf(stderr, "Warning: request %d is infeasible because upper bound of pick-up time window [%f, %f] is to early to reach pick-up from depot.", i, D.nodes[i].start_tw, D.nodes[i].end_tw);
            exit(-1);
        }
    }
    // Initialize feasibilty matrix for j = 0
    for (int i=1; i<=n; i++)
    {
        f[i][0][0] = 1;
        f[i][0][1] = 1;
        f[0][i][0] = 1;
        f[0][i][1] = 1;
    }
    
    for (const auto& i: D.R)
    {
        for (const auto& j: D.R)
        {
            if (j != i)
            {
                if (D.nodes[j].start_tw + D.nodes[j].service_time + D.tt[j][i] > D.nodes[i].end_tw || (D.nodes[i].demand + D.nodes[j].demand > D.veh_capacity))
                {
                    f[i][j][0] = 0;
                    f[i][j][1] = 0;
                }
                else
                {
                    // test path 0
                    // j --- i --- n+j --- n+i
                    path.start = j;
                    D.next_array[j] = i;
                    D.next_array[i] = n+j;
                    D.next_array[n+j] = n+i;
                    D.next_array[n+i] = -1; // mark the end of the path
                    path.end = n+i;

                    if (eight_step(D, path))
                        f[i][j][0] = 1;
                    else
                        f[i][j][0] = 0;
                    
                    // test path 1
                    // j --- i --- n+i --- n+j
                    path.start = j;
                    D.next_array[j] = i;
                    D.next_array[i] = n+i;
                    D.next_array[n+i] = n+j;
                    D.next_array[n+j] = -1;
                    path.end = n+j;

                    if (eight_step(D, path))
                        f[i][j][1] = 1;
                    else
                        f[i][j][1] = 0;  
                }
            }
        }
    }
}







bool DARPSolver:: eight_step(DARP& D, DARPRoute& path)
{
    ///
    /// MODIFIED TO CHECK feasibility of user pairs i, j
    /// evaluation of route veh using the 8-step evaluation scheme 
    /// by Cordeau and Laporte (2003)
    ///

    
    // Step 1 
    path.departure_depot = D.nodes[DARPH_DEPOT].start_tw;
      
    // Step 2 - compute A_i, W_i, B_i and D_i for each vertex v_i in the route
    // if infeasibility is detected during the computation of an "earliest possible"-schedule, return false
    if (!update_vertices(D, path))
        return false;
    
        
    // Step 3 - compute Forward Time Slack at depot
    double fts = DARPH_INFINITY;
    double waiting = 0;
    double temp;
    int c = path.start;
    
    while (c > 0)
    {
        if (c <= D.num_requests && D.nodes[c].ride_time > 0)
        {
            fprintf(stderr,"Ride time > 0 assigned to nodes[%d].ride_time", c);
            exit(-1);
        }
            
        waiting += D.nodes[c].waiting_time;
        temp =  waiting + DARPH_PLUS(DARPH_MIN(D.nodes[c].end_tw - D.nodes[c].beginning_service, 
                    D.nodes[c].max_ride_time - D.nodes[c].ride_time));
        if (temp < fts)
            fts = temp;  
        c = D.next_array[c];
    }
    // we have to account for time window and maximum duration of route in fts
    // no waiting time at depot --> we do not increase waiting 
    temp = waiting + DARPH_PLUS(DARPH_MIN(D.nodes[DARPH_DEPOT].end_tw - path.return_depot, 
            D.max_route_duration - (path.return_depot - path.departure_depot)));
    if (temp < fts)
        fts = temp;

    // Step 4
    path.departure_depot = D.nodes[DARPH_DEPOT].start_tw + DARPH_MIN(fts, waiting);

    // Step 5 - update A_i, W_i, B_i and D_i for each vertex v_i in the route
    update_vertices(D, path);

    // Step 6 - compute L_i for each request assigned to the route
    // this is done in update_vertices already
    
    // Step 7 
    // reset c 
    c = path.start;
    while (c > 0)
    {
        if (c <= D.num_requests)
        {
            
            // Step 7a) - compute F_c
            double fts = DARPH_INFINITY;
            double waiting = 0;
            double temp = DARPH_PLUS(DARPH_MIN(D.nodes[c].end_tw - D.nodes[c].beginning_service, D.nodes[c].max_ride_time - D.nodes[c].ride_time));
            fts = temp;
            int i = D.next_array[c];
            while (i > 0)
            {   
                waiting += D.nodes[i].waiting_time;
                temp = waiting + DARPH_PLUS(DARPH_MIN(D.nodes[i].end_tw - D.nodes[i].beginning_service, D.nodes[i].max_ride_time - D.nodes[i].ride_time));
                if (temp < fts)
                    fts = temp;
                i = D.next_array[i];
            }
            // we have to account for time window and maximum duration of route
            // no waiting time at depot --> we do not increase waiting 
            temp = waiting + DARPH_PLUS(DARPH_MIN(D.nodes[DARPH_DEPOT].end_tw - path.return_depot, 
                D.max_route_duration - (path.return_depot - path.departure_depot)));
            if (temp < fts)
                fts = temp;
            
            // Step 7b) - Set B_c = B_c + min(F_c, sum W_p); D_j = B_j + d_j
            D.nodes[c].beginning_service += DARPH_MIN(fts, waiting);
            D.nodes[c].departure_time = D.nodes[c].beginning_service + D.nodes[c].service_time;
            D.nodes[c].waiting_time = DARPH_MAX(0,D.nodes[c].beginning_service - D.nodes[c].arrival_time);

            // Step 7c) - update A_i, W_i, B_i and D_i for each vertex v_i that comes after v_c in the route
            update_vertices(D, path, c);

            // Step 7d) - update ride time L_i for each request whose destination vertex is after v_c
            // already done in update_vertices(veh, c)

        }

        c = D.next_array[c];
    }

    // Step 8 - compute violation route duration, time window and ride time constraints
    
    // duration violation
    if ((path.return_depot - path.departure_depot) > D.max_route_duration)
    {
        return false;    
    }
    
    // ride time violation and time window violation
    c = path.start;
    while(c>0)
    {
        if (c > D.num_requests && D.nodes[c].ride_time > D.nodes[c].max_ride_time)
        {
            return false;
        }
        if (D.nodes[c].beginning_service -0.001 > D.nodes[c].end_tw || D.nodes[c].beginning_service +0.001 < D.nodes[c].start_tw)
        {
            int e = path.start;
            std::cerr << std::endl;
            std::cerr << "Path: ";
            while (e>0)
            {
                std::cerr << e << " ";
                e = D.next_array[e];
            }
            std::cerr << std::endl;
            fprintf(stderr, "Problem in eight_step() routine. Routine should have stopped already due to time window infeasibility.");
            fprintf(stderr, "Node %d: Beginning of service is %f. Time window is [%f, %f].", c, D.nodes[c].beginning_service, D.nodes[c].start_tw, D.nodes[c].end_tw);
            exit(-1);
            return false;
        }

        c = D.next_array[c];
    }

    return true;
}   



bool DARPSolver::update_vertices(DARP& D, DARPRoute& path)
{
    ///
    /// Update A_i, W_i, B_i and D_i for each vertex v_i in the route 
    /// in 8-step routine
    ///

    int c = path.start; // c - current
    int s = D.next_array[c]; // s - successor
    D.nodes[c].arrival_time = path.departure_depot + D.tt[DARPH_DEPOT][c];
    D.nodes[c].beginning_service = DARPH_MAX(D.nodes[c].arrival_time, D.nodes[c].start_tw);
    D.nodes[c].waiting_time = DARPH_MAX(0,D.nodes[c].beginning_service - D.nodes[c].arrival_time);
    D.nodes[c].departure_time = D.nodes[c].beginning_service + D.nodes[c].service_time;
    // check for infeasibility of tws (this is the earliest possible schedule)
    if (D.nodes[c].beginning_service > D.nodes[c].end_tw + DARPH_EPSILON)
        return false; 

    // in the first iteration of the loop s should be positive because for each passenger 
    // there are at least two nodes (i and n+i) in the route 
    // when the end of the route is reached s is negative (new route) or zero (DARPH_DEPOT)
    while (s > 0)
    {
        D.nodes[s].arrival_time = D.nodes[c].departure_time + D.tt[c][s];
        D.nodes[s].beginning_service = DARPH_MAX(D.nodes[s].arrival_time, D.nodes[s].start_tw);
        D.nodes[s].waiting_time = DARPH_MAX(0,D.nodes[s].beginning_service - D.nodes[s].arrival_time);
        D.nodes[s].departure_time = D.nodes[s].beginning_service + D.nodes[s].service_time;

        // modification of routine to update ride times immedialely
        if (s >= D.num_requests + 1)
            D.nodes[s].ride_time = D.nodes[s].beginning_service - D.nodes[s - D.num_requests].departure_time;

        // check for infeasibility of tws (this is the earliest possible schedule)
        if (D.nodes[s].beginning_service > D.nodes[s].end_tw + DARPH_EPSILON)
            return false; 
        c = s; 
        s = D.next_array[c];
    } 
    path.return_depot = D.nodes[c].departure_time + D.tt[c][DARPH_DEPOT];
    return true;
}


bool DARPSolver::update_vertices(DARP& D, DARPRoute& path, int j)
{
    ///
    /// Update A_i, W_i, B_i and D_i for each vertex v_i that comes after v_j in the route 
    ///
    int c = j;
    int s = D.next_array[c]; // s - successor
        
    // in the first iteration of the loop s should be positive because c is a pick-up node 
    // when the beginning of a new route is reached s is negative or zero (DARPH_DEPOT)
    while (s > 0)
    {
        D.nodes[s].arrival_time = D.nodes[c].departure_time + D.tt[c][s];
        D.nodes[s].beginning_service = DARPH_MAX(D.nodes[s].arrival_time, D.nodes[s].start_tw);
        D.nodes[s].waiting_time = DARPH_MAX(0,D.nodes[s].beginning_service - D.nodes[s].arrival_time);
        D.nodes[s].departure_time = D.nodes[s].beginning_service + D.nodes[s].service_time;

        // modification of routine to update ride times immediately
        if (s >= D.num_requests + 1)
            D.nodes[s].ride_time = D.nodes[s].beginning_service - D.nodes[s - D.num_requests].departure_time;

        
        c = s;
        s = D.next_array[c];
    } 
    path.return_depot = D.nodes[c].departure_time + D.tt[c][DARPH_DEPOT];
    return true;
}








