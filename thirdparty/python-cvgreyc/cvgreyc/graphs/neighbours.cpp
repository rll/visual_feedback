#include "neighbours.h"
#include <queue>
#include <algorithm>
#include <utility>


using namespace std;

Matrix<size_t>*  create_ranks(const Matrix<double>& distances){
    Matrix<size_t>* ranks = new MatrixCArray<size_t>(distances.nrow(),distances.ncol());
    typedef pair< double , size_t > DistVert;
    vector<DistVert> s_verts;
    for (size_t i = 0; i< distances.nrow(); i++){
        s_verts.clear();
        for (size_t j = 0; j<distances.ncol(); j++){
            if (i==j) continue;
            s_verts.push_back(make_pair(distances(i,j),j));
        }
        sort(s_verts.begin(), s_verts.end());
        (*ranks)(i,i) = 0;
        for (size_t j = 0; j<s_verts.size(); j++){
            (*ranks)(i,s_verts[j].second) = j+1;
        }
    }
    return ranks;
}

EdgeSet* build_mst(const Matrix<double>& distances){
    typedef pair< double , Edge > DistEdge;
    priority_queue<DistEdge> pq;
    for (size_t i = 0; i<distances.nrow()-1; i++){
        for (size_t j = i+1 ; j < distances.ncol() ; j++){
            pq.push( make_pair(- distances(i,j), make_pair(i,j) ) );
        }
    }
    EdgeSet* es = new EdgeSet();
    ConnComp cc(distances.nrow());
    while (not pq.empty() and cc.ncc() >1 ){
        DistEdge de = pq.top();
        Edge e = de.second;
        size_t i = e.first;
        size_t j = e.second;
        if (cc.connect(i,j)) es->push_back(e);
        pq.pop();
    }
    return es;
}

EdgeSet* build_incr_knn(const Matrix<double>& distances){
    Matrix<size_t> *ranks = create_ranks(distances);
    typedef pair<long int , Edge > RankEdge;
    priority_queue<RankEdge> pq;
    for (size_t i = 0; i<distances.nrow()-1; i++){
        for (size_t j = i+1 ; j < distances.ncol() ; j++){
            size_t rij = (*ranks)(i,j);
            size_t rji = (*ranks)(j,i);
            pq.push( make_pair(- (rji < rij ? rji : rij) , make_pair(i,j) ) );
        }
    }
    EdgeSet* es = new EdgeSet();
    ConnComp cc(distances.nrow());
    long int curr_rank = 0;
    while (not pq.empty() and (cc.ncc() >1 or curr_rank == -pq.top().first) ){
        RankEdge de = pq.top();
        curr_rank = - de.first;
        Edge e = de.second;
        size_t i = e.first;
        size_t j = e.second;
        cc.connect(i,j);
        es->push_back(e);
        pq.pop();
    }
    delete ranks;
    return es;   
}

EdgeSet* build_incr_mknn(const Matrix<double>& distances){
    Matrix<size_t> *ranks = create_ranks(distances);
    typedef pair<long int , Edge > RankEdge;
    priority_queue<RankEdge> pq;
    for (size_t i = 0; i<distances.nrow()-1; i++){
        for (size_t j = i+1 ; j < distances.ncol() ; j++){
            size_t rij = (*ranks)(i,j);
            size_t rji = (*ranks)(j,i);
            pq.push( make_pair(- (rji > rij ? rji : rij) , make_pair(i,j) ) );
        }
    }
    EdgeSet* es = new EdgeSet();
    ConnComp cc(distances.nrow());
    long int curr_rank = 0;
    while (not pq.empty() and (cc.ncc() >1 or curr_rank == -pq.top().first) ){
        RankEdge de = pq.top();
        curr_rank = - de.first;
        Edge e = de.second;
        size_t i = e.first;
        size_t j = e.second;
        cc.connect(i,j);
        es->push_back(e);
        pq.pop();
    }
    delete ranks;
    return es;   
}

EdgeSet* build_knn(const Matrix<double>& distances, size_t k){
    Matrix<size_t> *ranks = create_ranks(distances);
    EdgeSet* es = new EdgeSet();
    for (size_t i = 0; i<distances.nrow()-1; i++){
        for (size_t j = i+1 ; j < distances.ncol() ; j++){
            size_t rij = (*ranks)(i,j);
            size_t rji = (*ranks)(j,i);
            if ((rji < rij ? rji : rij) <= k ) es->push_back(make_pair(i,j));
        }
    }
    delete ranks;
    return es;   
}

EdgeSet* build_mknn(const Matrix<double>& distances, size_t k){
    Matrix<size_t> *ranks = create_ranks(distances);
    EdgeSet* es = new EdgeSet();
    for (size_t i = 0; i<distances.nrow()-1; i++){
        for (size_t j = i+1 ; j < distances.ncol() ; j++){
            size_t rij = (*ranks)(i,j);
            size_t rji = (*ranks)(j,i);
            if ((rji > rij ? rji : rij) <= k ) es->push_back(make_pair(i,j));
        }
    }
    delete ranks;
    return es;
}

EdgeSet* build_incr_eps(const Matrix<double>& distances){
    typedef pair<double , Edge > DistEdge;
    priority_queue<DistEdge> pq;
    for (size_t i = 0; i<distances.nrow()-1; i++){
        for (size_t j = i+1 ; j < distances.ncol() ; j++){
            pq.push( make_pair(- distances(i,j) , make_pair(i,j) ) );
        }
    }
    EdgeSet* es = new EdgeSet();
    ConnComp cc(distances.nrow());
    while (not pq.empty() and cc.ncc() >1){
        DistEdge de = pq.top();
        Edge e = de.second;
        size_t i = e.first;
        size_t j = e.second;
        cc.connect(i,j);
        es->push_back(e);
        pq.pop();
    }
    return es;    
}

EdgeSet* build_eps(const Matrix<double>& distances,double eps){
    EdgeSet* es = new EdgeSet();
    for (size_t i = 0; i<distances.nrow()-1; i++){
        for (size_t j = i+1 ; j < distances.ncol() ; j++){
            
            if (distances(i,j)<=eps) es->push_back(make_pair(i,j));
        }
    }
    return es;    
}
