#include "DARPH.h"


DARP::DARP(int n) : num_requests{n}, num_nodes{2*n} {
    ///
    /// Constructor for an n-node problem
    ///

    // Set these to default values--they may change once we read the file
    num_vehicles = DARPH_INFINITY;
    max_route_duration = DARPH_INFINITY;

    d = new double* [num_nodes+1];
    d[0] = new double [(num_nodes+1)*(num_nodes+1)];
    for(int i = 1; i < num_nodes+1; i++)
        d[i] = d[i-1] + (num_nodes+1) ;

    tt = new double* [num_nodes+1];
    tt[0] = new double [(num_nodes+1)*(num_nodes+1)];
    for(int i = 1; i < num_nodes+1; i++)
        tt[i] = tt[i-1] + (num_nodes+1) ;
    

    nodes = new DARPNode[num_nodes+1];
    
    next_array = new int[num_nodes+1];
    pred_array = new int[num_nodes+1];
    route_num = new int[num_nodes+1];
    routed = new bool[num_nodes+1];
    for (int i = 0; i < num_nodes+1; i++)
    {
        routed[i] = false;
    }

    // Allocate memory for route array when problem is loaded 
    route = NULL;

    for (int i=1; i<=num_requests; ++i)
    {
        R.push_back(i);
    }
}


DARP::~DARP() {  
    delete [] d[0];
    delete [] d;
    delete [] tt[0];
    delete [] tt;
    delete [] nodes;
    delete [] next_array;
    delete [] pred_array;
    delete [] route_num;
    delete [] routed;
    delete [] route;
}


DARP::DARP(DARP& D)
{
    instance_mode = D.instance_mode;

    num_requests = D.num_requests;
    num_nodes = D.num_nodes;
    num_vehicles = D.num_vehicles;
    max_route_duration = D.max_route_duration;
    veh_capacity = D.veh_capacity;
    
    
    // allocate memory for distance matrix
    d = new double* [num_nodes+1];
    d[0] = new double [(num_nodes+1)*(num_nodes+1)];
    for(int i = 1; i < num_nodes+1; i++)
        d[i] = d[i-1] + (num_nodes+1);

    // allocate memory for travel time matrix
    tt = new double* [num_nodes+1];
    tt[0] = new double [(num_nodes+1)*(num_nodes+1)];
    for(int i = 1; i < num_nodes+1; i++)
        tt[i] = tt[i-1] + (num_nodes+1) ;

    // copy values of D.d into d and D.tt into tt
    for (int i=0; i<=num_nodes; ++i)
    {
        for (int j=0; j<=num_nodes; ++j)
        {
            d[i][j] = D.d[i][j];
            tt[i][j] = D.tt[i][j];
        }
    }

    // allocate memory for other arrays
    nodes = new DARPNode[num_nodes+1];
    next_array = new int[num_nodes+1];
    pred_array = new int[num_nodes+1];
    route_num = new int[num_nodes+1];
    routed = new bool[num_nodes+1];
    
    // copy values
    for (int i=0; i<=num_nodes; ++i)
    {
        nodes[i] = D.nodes[i];
        next_array[i] = D.next_array[i];
        pred_array[i] = D.pred_array[i];
        route_num[i] = D.route_num[i];
        routed[i] = D.routed[i];
    }
    
    route = new DARPRoute[num_vehicles];
    for (int k=0; k<num_vehicles; ++k)
    {
        route[k] = D.route[k];
    }

    R = D.R;
}

void DARP::preprocess()
{
    ///
    /// this functions tightens the time windows of the non-critical vertex 
    /// according to Cordeau [2006]
    ///
    int n = num_requests;
    
    for (int i = 1; i<=n; ++i)
    {
        
        if (nodes[i].start_tw < DARPH_EPSILON && nodes[i].end_tw >= max_route_duration) 
        {
            nodes[i].end_tw = DARPH_MAX(0,nodes[n+i].end_tw - tt[i][n+i]- nodes[i].service_time);
            nodes[i].start_tw = DARPH_MAX(0,nodes[n+i].start_tw - nodes[i].max_ride_time - nodes[i].service_time);
            if (nodes[i].end_tw <= nodes[i].start_tw)
            {
                fprintf(stderr, "%s: Time window preprocessing at node %d leads to error in time windows.\n",__FUNCTION__, i);
                exit(-1);
            }
        }
        if (nodes[n+i].start_tw < DARPH_EPSILON && nodes[n+i].end_tw >= max_route_duration)
        {
            nodes[n+i].start_tw = nodes[i].start_tw + nodes[i].service_time + tt[i][n+i];
            nodes[n+i].end_tw = nodes[i].end_tw + nodes[i].service_time + nodes[i].max_ride_time;
        }
    }
}



void DARP::read_file(std::string infile, std::string data_directory, std::string instance)
{   
    ///
    /// Currently reads file in format of the pr-set (Cordeau and Laporte, 2003).
    /// For another type of test instance pay attention to nodes[i].max_ride_time.
    ///

    double temp_max_ride_time;
    int i, j;
    double val;

    std::ifstream file;
    std::string line;
    file.open(infile.c_str(), std::ios_base::in);
    if (!file)
    {
        fprintf(stderr,"%s: file error\n", __FUNCTION__);
        exit(-1);
    }

    // Read first line of file, which contains the following data:
    // number of vehicles(K), number of nodes(N), maximum route duration, vehicle capacity(Q), maximum ride time(L) 
    getline(file,line);
    std::istringstream f(line);
    f >> num_vehicles >> num_nodes >> max_route_duration >> veh_capacity >> temp_max_ride_time;
    
    if (instance_mode == 1)
    {
        // The following lines have to be changed if there are individual maximum ride times
        for (i=1; i<=num_nodes; ++i)
        {
            nodes[i].max_ride_time = temp_max_ride_time;
        }
        
        // Read in the next lines until EOF is reached, which contain the data
        // id x y service_time load start_tw end_tw
        i = 0;
        while(i<=num_nodes)
        {
            std::getline(file,line);
            std::istringstream iss(line);
            iss >> nodes[i].id >> nodes[i].x >> nodes[i].y >> nodes[i].service_time >> nodes[i].demand >> nodes[i].start_tw >> nodes[i].end_tw;
            i++;
        }
        for (i=1; i<=num_requests/2; ++i)
        {
            // first half of requests is outbound
            nodes[i].tw_length = nodes[num_requests+i].end_tw - nodes[num_requests+i].start_tw;
            nodes[num_requests+i].tw_length = nodes[i].tw_length;
        }
        for (i=num_requests/2+1; i<=num_requests; ++i)
        {
            // second half of requests is inbound
            nodes[i].tw_length = nodes[i].end_tw - nodes[i].start_tw;
            nodes[num_requests+i].tw_length = nodes[i].tw_length;
        }
    }
    else
    {
        // Read in the next lines until EOF is reached, which contain the data
        // id service_time load start_tw end_tw max_ride_time
        i = 0;
        while(i<=num_nodes)
        {
            std::getline(file,line);
            std::istringstream iss(line);
            iss >> nodes[i].id >> nodes[i].service_time >> nodes[i].demand >> nodes[i].start_tw >> nodes[i].end_tw >> nodes[i].max_ride_time;
            i++;
        }
        // all requests are inbound
        for (i=1; i<=num_requests; i++)
        {
            nodes[i].tw_length = nodes[i].end_tw - nodes[i].start_tw;
            nodes[num_requests+i].tw_length = nodes[i].tw_length; 
        }  
    }
   
    file.close();

    // check if requested load is greater than the vehicle capacity
    for (i=0; i<=num_requests; ++i)
    {
        if (nodes[i].demand > veh_capacity)
        {
            fprintf(stderr, "Problem instance is infeasible due to excess load: demand of request %d is %d, vehicle capacity is %d\n", i, nodes[i].demand, veh_capacity);
            exit(-1);
        }
        if (nodes[num_requests+i].demand < -veh_capacity)
        {
            fprintf(stderr, "Problem instance is infeasible due to excess load: demand of request %d is %d, vehicle capacity is %d\n", i, nodes[num_requests+i].demand, veh_capacity);
            exit(-1);
        }
    }
   
    // Memory for route array is allocated
    route = new DARPRoute[num_vehicles];
    
    // Create distance and travel time matrix 
    if (instance_mode == 1)
    {
        for(i=0; i<=num_nodes; ++i)
        {
            for(j=0; j<=num_nodes; j++)
            {
                val = sqrt((nodes[i].x - nodes[j].x) * (nodes[i].x - nodes[j].x) 
                        + (nodes[i].y - nodes[j].y) * (nodes[i].y - nodes[j].y));
                d[i][j] = roundf(val * 100) / 100; 
                tt[i][j] = d[i][j];
            }  
        }
    }    
    else
    {
        std::string path_to_costs = data_directory + instance + "_c_a.txt";

        file.open(path_to_costs.c_str(), std::ios_base::in);
        if (!file)
        {
            fprintf(stderr, "%s: costs file error\n", __FUNCTION__);
            exit(-1);
        }
            

        for(i=0; i<num_nodes+1; ++i)
        {
            std::getline(file,line);
            std::istringstream iss(line);
            for(j=0; j<num_nodes+1; j++)
            {
                iss >> d[i][j];
                val = 1.8246 * d[i][j] + 2.3690; // based on linear regression with data = all completed rides (Jan, Feb 21)
                tt[i][j] = roundf(val * 100) / 100;
            }    
        }
        file.close();
    } 
    
    return;
}



