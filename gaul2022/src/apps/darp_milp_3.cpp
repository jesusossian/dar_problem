#include "DARPH.h"

int main(int argc,char* argv[]) 
{
    bool accept_all = false;
    bool consider_excess_ride_time = true;
    
    std::string instance(argv[1]);
    const std::string data_directory = "data/a_b_first_line_modified/";
    std::string path_to_instance = data_directory + instance + ".txt";

    int num_requests = DARPGetDimension(path_to_instance)/2;
    auto D = DARP(num_requests);

    // switch between different types of instances
    // 1: a-b instances
    // 2: other instances
    if (argv[1][0] == 'a' || argv[1][0] =='b')
        D.instance_mode = 1;
    else
        D.instance_mode = 2;

    D.read_file(path_to_instance, data_directory, argv[1]);
    if (D.instance_mode == 1)
    {
        // tighten time windows
        D.preprocess();
    }
    
    auto G = DARPGraph<3>(num_requests);
    auto EB = EBMILP<3>(num_requests);
    
    if (D.instance_mode == 1)
    {
        accept_all = true;
        consider_excess_ride_time = false;
    }
   
    EB.solve(accept_all, consider_excess_ride_time, D, G);
    
    // solve other MILP formulations (only routing costs implemented as objective function)
    //EB.solve_furtado(D,G);
    //EB.solve_cordeau(D,G);
    //EB.solve_milpI(D,G);    
    
    return 0;
}

