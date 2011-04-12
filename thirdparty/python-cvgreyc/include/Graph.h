#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <valarray>
#include <list>
#include <utility>


struct Graph {
    typedef std::pair<size_t, double>              neighbour;
    typedef std::list<neighbour>                   neighbour_list;
    typedef std::list<neighbour>::const_iterator   neighbour_iterator;
    
    size_t nnodes;
    std::valarray< neighbour_list > neighbours;
    
    
    
    Graph(size_t _nnodes);
    ~Graph();

    void  addEdge(size_t i, size_t j, double weight=1.0, bool symmetric = false); 
};


#endif

