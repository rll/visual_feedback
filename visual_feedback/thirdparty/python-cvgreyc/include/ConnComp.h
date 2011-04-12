#ifndef __CONN_COMP_H__
#define __CONN_COMP_H__

#include <list>
#include <valarray>

class ConnComp {
public:
    typedef std::list< size_t >   Cluster;
    typedef std::list< Cluster >  ClusterSet;

private:
    size_t _size;
    size_t _ncc; 
    std::valarray<size_t> _nodes;
    std::valarray<size_t> _ranks;
public:
    ConnComp(size_t size);
    bool connect(size_t i, size_t j);
    size_t find(size_t i);
    size_t ncc() const { return _ncc; }
    ClusterSet* clusters() const;
};


inline ConnComp::ConnComp(size_t size):_size(size),_ncc(size){
    _nodes.resize(size);
    _ranks.resize(size);
    _ranks = 0;
    for (size_t i = 0; i<_nodes.size() ; i++) _nodes[i] = i;
}

inline size_t ConnComp::find(size_t i){
    if (_nodes[i]!=i) _nodes[i] = find(_nodes[i]);
    return _nodes[i];
}

inline bool ConnComp::connect(size_t i, size_t j){
    size_t pi = find(i);
    size_t pj = find(j);
    if (pi == pj) return false;
    if( _ranks[pi] > _ranks[pj]) _nodes[pj] = _nodes[pi];
    else {
       _nodes[pi] = pj;
       if (_ranks[pi] == _ranks[pj]) _ranks[pj]+=1 ;
    }
    _ncc-=1;
    return true;
}

inline ConnComp::ClusterSet* ConnComp::clusters() const {
    ClusterSet* clusts = new ClusterSet();
    std::valarray<Cluster> tmp(_size);
    
    for (size_t  i = 0 ; i< _nodes.size() ; i++){
        tmp[_nodes[i]].push_back(i);
    }
    
    for (size_t i = 0 ; i< _nodes.size() ; i++){
        if (tmp[i].size() > 0) (*clusts).push_back(tmp[i]);
    }
    return clusts;
}
#endif

