#pragma once

#include <wmtk/ConcurrentTriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <queue>
#include <tbb/concurrent_vector.h>
#include <tbb/task_group.h>
#include <tbb/parallel_for.h>
#include <tbb/concurrent_priority_queue.h>
#include <tbb/task_group.h>


namespace Edge2d {

class ElementInQueue
{
public:
    wmtk::TriMesh::Tuple edge;
    double weight;

    ElementInQueue() {}
    ElementInQueue(const wmtk::TriMesh::Tuple& e, double w)
        : edge(e)
        , weight(w)
    {}
};

struct cmp_l
{
    bool operator()(const ElementInQueue& e1, const ElementInQueue& e2)
    {
        if (e1.weight == e2.weight) return e1.edge.get_vid() > e2.edge.get_vid();
        return e1.weight < e2.weight;
    }
};

struct cmp_s
{
    bool operator()(const ElementInQueue& e1, const ElementInQueue& e2)
    {
        if (e1.weight == e2.weight) return e1.edge.get_vid() < e2.edge.get_vid();
        return e1.weight > e2.weight;
    }
};

class EdgeCollapse : public wmtk::TriMesh
{
public:
    tbb::concurrent_vector<Eigen::Vector3d> m_vertex_positions;

    EdgeCollapse(tbb::concurrent_vector<Eigen::Vector3d> _m_vertex_positions)
        : m_vertex_positions(_m_vertex_positions)
    {}

    ~EdgeCollapse() {}

    bool collapse_shortest();
    void collapse_shortest_stuff(ElementInQueue &eiq, tbb::concurrent_priority_queue<ElementInQueue, cmp_s> &ec_queue);

    bool collapse_qec();
    // get the quadrix in form of an array of 10 floating point numbers
    std::array<double, 10> compute_Q_f(wmtk::TriMesh::Tuple& t);

    std::array<double, 10> compute_Q_v(wmtk::TriMesh::Tuple& t);

    void update_position(size_t v1, size_t v2, Tuple& new_vert);



    // class parallel_collapse_shortest: public wmtk::TriMesh{
    // private:
    //     tbb::concurrent_priority_queue<ElementInQueue, cmp_s> &ec_queue;
    //     tbb::concurrent_vector<Eigen::Vector3d> &m_vertex_positions;
    // public:
    //     parallel_collapse_shortest(tbb::concurrent_priority_queue<ElementInQueue, cmp_s> &eq, \
    //     tbb::concurrent_vector<Eigen::Vector3d> &_m_vertex_positions): ec_queue(eq), m_vertex_positions(_m_vertex_positions){}
        
    //     void update_position(size_t v1, size_t v2, Tuple& new_vert) {
    //         size_t new_vid = new_vert.get_vid();
    //         Eigen::Vector3d new_position = (m_vertex_positions[v1] + m_vertex_positions[v2]) / 2;
    //         if (new_vid < m_vertex_positions.size()) m_vertex_positions[new_vid] = new_position;
    //         m_vertex_positions.emplace_back(new_position);
    //     }

    //     void operator()(const tbb::blocked_range<size_t>& r) {
    //         for (size_t i=r.begin(); i!=r.end(); ++i){
    //             ElementInQueue eiq;
    //             while (ec_queue.try_pop(eiq)) {
    //                 auto loc = eiq.edge;
    //                 double weight = eiq.weight;
    //                 // check if the edge tuple is valid
    //                 if (!loc.is_valid(*this)) continue;
    //                 //set lock here
    //                 size_t v1 = loc.get_vid();
    //                 TriMesh::Tuple v2_tuple = loc.switch_vertex(*this);
    //                 size_t v2 = v2_tuple.get_vid();
    //                 TriMesh::Tuple new_vert;
    //                 if (!TriMesh::collapse_edge(loc, new_vert)) continue;
    //                 update_position(v1, v2, new_vert);
    //                 size_t new_vid = new_vert.get_vid();
    //                 std::vector<TriMesh::Tuple> one_ring_edges = get_one_ring_edges_for_vertex(new_vert);
    //                 for (TriMesh::Tuple edge : one_ring_edges) {
    //                     TriMesh::Tuple tmp_tuple = switch_vertex(new_vert);
    //                     size_t vid = tmp_tuple.get_vid();
    //                     double length = (m_vertex_positions[new_vid] - m_vertex_positions[vid]).squaredNorm();
    //                     ec_queue.push(ElementInQueue(edge, length));
    //                 }
    //             }
    //         }
    //     }
    // };

};




}// namespace Edge2d