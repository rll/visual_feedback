#include "Graph.h"

using namespace std;

Graph::Graph(size_t _nnodes):nnodes(_nnodes){
    neighbours.resize(nnodes);
}
Graph::~Graph(){};

void Graph::addEdge(size_t i, size_t j, double weight, bool symmetric){
    neighbours[i].push_back(neighbour(j,weight));
    if (symmetric) neighbours[j].push_back(neighbour(i,weight));
}

