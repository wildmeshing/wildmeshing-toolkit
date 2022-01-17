#pragma once
#include <igl/write_triangle_mesh.h>
#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <queue>


namespace Edge2d {

class EdgeOperations2d : public wmtk::TriMesh
{
public:
    std::vector<Eigen::Vector3d> m_vertex_positions;

    EdgeOperations2d(std::vector<Eigen::Vector3d> _m_vertex_positions)
        : m_vertex_positions(_m_vertex_positions)
    {}

    ~EdgeOperations2d() {}

    struct PositionInfoCache
    {
        Eigen::Vector3d v1p;
        Eigen::Vector3d v2p;
    } position_cache;

    void cache_edge_positions(const Tuple& t)
    {
        position_cache.v1p = m_vertex_positions[t.vid()];
        position_cache.v2p = m_vertex_positions[t.switch_vertex(*this).vid()];
    }

    void update_position_to_edge_midpoint(const Tuple& t)
    {
        m_vertex_positions[t.vid()] = (position_cache.v1p + position_cache.v2p) / 2.0;
    }

    bool smooth(const Tuple& t)
    {
        auto one_ring_edges = get_one_ring_edges_for_vertex(t);
        if (one_ring_edges.size() < 3) return false;
        Eigen::Vector3d after_smooth(0, 0, 0);
        Eigen::Vector3d after_smooth_boundary(0, 0, 0);
        bool boundary = false;
        for (auto e : one_ring_edges) {
            if (is_boundary_edge(e)) {
                after_smooth_boundary += m_vertex_positions[e.vid()];
                boundary = true;
            }
            after_smooth += m_vertex_positions[e.vid()];
        }
        if (boundary)
            m_vertex_positions[t.vid()] = after_smooth_boundary / 2.0;
        else
            m_vertex_positions[t.vid()] = after_smooth / one_ring_edges.size();
        return true;
    }

    void move_vertex_attribute(size_t from, size_t to) override
    {
        m_vertex_positions[to] = m_vertex_positions[from];
    }

    // write the collapsed mesh into a obj
    bool write_triangle_mesh(std::string path)
    {
        Eigen::MatrixXd V = Eigen::MatrixXd::Zero(m_vertex_positions.size(), 3);
        for (auto& t : get_vertices()) {
            auto i = t.vid();
            V.row(i) = m_vertex_positions[i];
        }

        Eigen::MatrixXi F = Eigen::MatrixXi::Constant(tri_capacity(), 3, -1);
        for (auto& t : get_faces()) {
            auto i = t.fid();
            auto vs = oriented_tri_vertices(t);
            for (int j = 0; j < 3; j++) {
                F(i, j) = vs[j].vid();
            }
        }

        return igl::write_triangle_mesh(path, V, F);
    }

    bool collapse_before(const Tuple& t) override
    {
        if (!TriMesh::collapse_before(t)) return false;
        cache_edge_positions(t);
        return true;
    }

    bool collapse_after(const Tuple& t) override
    {
        update_position_to_edge_midpoint(t);
        return true;
    }

    std::vector<TriMesh::Tuple> new_edges_after_collapse_split(const TriMesh::Tuple& t) const;
    std::vector<TriMesh::Tuple> new_edges_after_swap(const TriMesh::Tuple& t) const;

    bool collapse_shortest(int target_vertex_count);

    bool collapse_qec();
    // get the quadrix in form of an array of 10 floating point numbers
    std::array<double, 10> compute_Q_f(wmtk::TriMesh::Tuple& t);

    std::array<double, 10> compute_Q_v(wmtk::TriMesh::Tuple& t);

    double compute_cost_for_v(wmtk::TriMesh::Tuple& v_tuple);

    bool split_before(const Tuple& t) override
    {
        cache_edge_positions(t);
        return true;
    }

    bool split_after(const Tuple& t) override
    {
        update_position_to_edge_midpoint(t);
        return true;
    }
    // methods for adaptive remeshing
    double compute_edge_cost_collapse_ar(const TriMesh::Tuple& t, double L);
    double compute_edge_cost_split_ar(const TriMesh::Tuple& t, double L);
    double compute_vertex_valence_ar(const TriMesh::Tuple& t);
    std::pair<double, double> average_len_valen();
    bool split_remeshing(double L);
    bool collapse_remeshing(double L);
    bool swap_remeshing();
    bool adaptive_remeshing(double L, int interations);
    void resize_vertex_attributes(size_t v) override { m_vertex_positions.resize(v); }
};

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
        if (e1.weight == e2.weight) return e1.edge.vid() > e2.edge.vid();
        return e1.weight < e2.weight;
    }
};
struct cmp_s
{
    bool operator()(const ElementInQueue& e1, const ElementInQueue& e2)
    {
        if (e1.weight == e2.weight) return e1.edge.vid() < e2.edge.vid();
        return e1.weight > e2.weight;
    }
};

} // namespace Edge2d
