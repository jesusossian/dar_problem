#include "DARPH.h"

template<int Q>
EBMILP<Q>::EBMILP(int num_requests) : DARPSolver{num_requests}  
{
    vec_map = new std::unordered_map<NODE,int,HashFunction<Q>>[n]; 
}

template<int Q>
EBMILP<Q>::~EBMILP() {
    delete[] vec_map;
}


template<int Q>
void EBMILP<Q>::create_maps(DARP& D, DARPGraph<Q>& G) {

    uint64_t count = 0;
    uint64_t count_vout = 0;
    
    // add depot node to vmap
    vmap[G.depot] = count;
    count ++;
    for (const auto& v: G.V_in)
    {
        vmap[v] = count;
        vinmap[v] = count - 1;
        count ++;

        G.V_i[v[0]].push_back(v);
    }
    G.vincardinality = count - 1;
    count_vout = 0;
    for (const auto& v: G.V_out)
    {
        vmap[v] = count;
        voutmap[v] = count_vout;
        count ++;
        count_vout ++;

        G.V_i[v[0]].push_back(v);
    }
    G.vcardinality = count;
    G.voutcardinality = count_vout; 
    

       
    // for each request create an unordered map that maps each node in V_{n+i} to an integer
    for (const auto& i: D.R)
    {
        count = 0;
        for (const auto& w : G.V_i[n+i])
        {
            vec_map[i-1][w] = count;
            count++;
        }
    }
    
    // create arc map
    count = 0;
    for (const auto& a: G.A)
    {
        amap[a] = count;
        count ++;
    }
    G.acardinality = count; 

     // initialize rmap
    count = 0;
    for (const auto & i: D.R)
    {
        rmap[i] = count;
        count++;
    }
    
}

template<int Q>
void EBMILP<Q>::get_solution_values(bool consider_excess_ride_time, DARP& D, DARPGraph<Q>& G)
{
  
    // If CPLEX successfully solved the model, print the results
    std::cout << "\nCplex success!\n";
    std::cout << "\tStatus: " << cplex.getStatus() << "\n";
    std::cout << "\tObjective value: " << cplex.getObjValue() << "\n";
#if VERBOSE    
    std::cout << "\tRelative MIP Gap: " << cplex.getMIPRelativeGap() << "\n";
    std::cout << "\tModel MILP " << dur_model.count() << "s" << std::endl;
    std::cout << "\tModel + Solve MILP " << dur_solve.count() << "s" << std::endl;  
#endif

    
    // get solution values
    B_val[vmap[G.depot]] = cplex.getValue(B[vmap[G.depot]]);
    for (const auto& v: G.V_in)
    {
        B_val[vmap[v]] = cplex.getValue(B[vmap[v]]);
    }
    for (const auto& v: G.V_out)
    {
        B_val[vmap[v]] = cplex.getValue(B[vmap[v]]);
    }
    for (const auto& a: G.A)
    {
        if (cplex.getValue(x[amap[a]]) > 0.9)
        {
            x_val[amap[a]] = 1;
        }
        else 
        {
            x_val[amap[a]] = 0;
        }
    }
    for (const auto& i : D.R)
    {
        
        if (cplex.getValue(p[rmap[i]]) > 0.9)
        {
            p_val[rmap[i]] = 1;
        }
        else
        {
            p_val[rmap[i]] = 0;
        }
        if (consider_excess_ride_time)
        {
            d_val[rmap[i]] = cplex.getValue(d[rmap[i]]);
        }
    }
#if VERBOSE
    print_routes(D, G);
#endif
    
    total_time_model += dur_model.count();
    total_time_model_solve += dur_solve.count();  
}


template<int Q>
void EBMILP<Q>::print_routes(DARP& D, DARPGraph<Q>& G)
{
    // output solution we have so far

    bool flag;
    int route_count = 0;    
    int current;

    std::vector<ARC> cycle_arcs;
    std::vector<ARC> cycle;
    ARC b = {};
    
    for (const auto& a: G.A)
    {
        if (x_val[amap[a]] > 0.9)
        {
            cycle_arcs.push_back(a);
        }
    }

    while (!cycle_arcs.empty())
    {
        std::cout << std::endl;
        flag = false;
        for (const auto& a: cycle_arcs)
        {
            if (a[0] == G.depot)
            {
                std::cout << std::left << setw(STRINGWIDTH) << setfill(' ') << "Start";
                b = a;
                cycle_arcs.erase(std::remove(cycle_arcs.begin(), cycle_arcs.end(), a), cycle_arcs.end());
                
                current = b[1][0];
                D.route[route_count].start = current;
                D.route[route_count].has_customers = true;
                D.routed[current] = true;
                D.route_num[current] = route_count;
                if (route_count == 0)
                {
                    D.next_array[DARPH_DEPOT] = -current;
                    D.pred_array[current] = DARPH_DEPOT;
                }
                else
                {
                    D.next_array[D.route[route_count-1].end] = -current;
                    D.pred_array[current] = -D.route[route_count-1].end;
                }
                // as long as we haven't found the last arc in the dicycle, look for the next arc in the dicycle and
                // add it to the tour
                while (b[1] != G.depot)
                {
                    for (const auto& f: cycle_arcs)
                    {
                        if (f[0] == b[1])
                        {
                            cycle.push_back(f);
                            if (f[0][0] > n)
                            {
                                std::cout << std::left << setw(STRINGWIDTH) << setfill(' ') << "out:";
                                std::cout << std::left << setw(STRINGWIDTH) << setfill(' ') << f[0][0] - n;
                            }
                            else
                            {
                                std::cout << std::left << setw(STRINGWIDTH) << setfill(' ') << "in:";
                                std::cout << std::left << setw(STRINGWIDTH) << setfill(' ') << f[0][0];
                            }
                            
                            if (f[1] != G.depot)
                            {
                                D.next_array[current] = f[1][0];
                                D.pred_array[f[1][0]] = current;
                                current = f[1][0];
                                D.routed[current] = true;
                                D.route_num[current] = route_count;
                            }
                    
                            for (int i = 0; i<2; ++i)
                            {
                                for (int j= 0; j<D.veh_capacity; ++j)
                                {
                                    b[i][j] = f[i][j];
                                }
                            }
                            cycle_arcs.erase(std::remove(cycle_arcs.begin(), cycle_arcs.end(), f), cycle_arcs.end());
                            break; 
                        }
                    }
                }

                D.route[route_count].end = current;
                D.next_array[current] = DARPH_DEPOT;

                std::cout << std::endl;
                std::cout << std::left << setw(STRINGWIDTH) << setfill(' ') << D.nodes[0].start_tw;
                for (const auto& a: cycle)
                {
                    std::cout << std::left << setw(STRINGWIDTH) << setfill(' ') << "Time:";
                    std::cout << std::left << setw(STRINGWIDTH) << setfill(' ') << B_val[vmap[a[0]]];
                    D.nodes[a[0][0]].beginning_service = B_val[vmap[a[0]]];
                }
                std::cout << std::endl;
                cycle.clear();
                flag = true;
                route_count++;
            }
            if (flag)
                break;
        }
    }
    D.pred_array[DARPH_DEPOT] = -D.route[route_count-1].end;
    std::cout << std::endl;
    cycle_arcs.clear();
}

template class EBMILP<3>;
template class EBMILP<6>;