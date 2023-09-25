#ifndef _DARP_H
#define _DARP_H

class DARP {   
public:
    int instance_mode;
    
    // Problem parameters, description, etc.
    int num_requests;
    int num_nodes; // without depot
    int num_vehicles; 
    double max_route_duration; // max duration of service
    int veh_capacity;
    double **d; // The distance matrix d
    double **tt; // The travel times matrix tt 

    class DARPNode *nodes; // Array of nodes - contains coordinates, time windows, load

    // Solution storage
    class DARPRoute *route; // Array stores useful information about the routes in a solution   
    int *next_array;
    int *pred_array;
    int *route_num;
    bool *routed; // Indicates wether the user is in a route yet or not

    std::vector<int> R;     
    
    DARP(int); // Constructor for an n-node problem
    ~DARP(); // Destructor
    DARP(DARP& D); // Copy Constructor

    // // file processing
    void read_file(std::string infile, std::string data_directory, std::string instance);
    // tighten time windows if necessary
    void preprocess();
    

    template<int Q>
    friend class DARPGraph;
    template<int Q>
    friend class EBMILP;
    friend class DARPSolver;
};

#endif