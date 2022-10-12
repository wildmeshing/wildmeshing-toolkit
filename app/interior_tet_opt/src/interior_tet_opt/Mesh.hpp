#pragma once

#include <wmtk/TetMesh.h>
#include <Eigen/Core>
#include <string>

namespace app::interior_tet_opt {
struct VertexAttribute
{
    bool freeze = false;
    Eigen::Vector3d pos;
    size_t partition_id = 0;
    double m_sizing_scalar = 1.;
};

struct TetraAttribute
{
    double quality = -1.;
};


struct InteriorTetOpt : wmtk::TetMesh
{
    InteriorTetOpt() {}
    ~InteriorTetOpt() {}
    double target_l = 5e-2;
    double m_splitting_l2 = -1.;
    double m_collapsing_l2 = -1.;
    void initialize(const std::vector<Eigen::Vector3d>&, const std::vector<std::array<size_t, 4>>&);

    wmtk::AttributeCollection<VertexAttribute> m_vertex_attribute;
    wmtk::AttributeCollection<TetraAttribute> m_tet_attribute;

    bool is_inverted(const Tuple&) const;
    double get_quality(const Tuple&) const;
    double get_length2(const Tuple&) const;
    bool invariants(const std::vector<Tuple>& t) override; //
    void final_output_mesh(std::string);

    struct SplitInfoCache
    {
        size_t v1_id = -1;
        size_t v2_id = -1;
        bool bnd = false;
        double max_quality = -1.;
    };
    tbb::enumerable_thread_specific<SplitInfoCache> split_cache;

    struct CollapseInfoCache
    {
        size_t v1_id;
        size_t v2_id;
        double max_energy;
        double edge_length;
        bool is_limit_length;

        std::vector<std::array<size_t, 3>> surface_faces;
        std::vector<size_t> changed_tids;

        std::vector<std::array<size_t, 2>> failed_edges;
    };
    tbb::enumerable_thread_specific<CollapseInfoCache> collapse_cache;

    bool split_edge_before(const Tuple& loc0) override;

    bool split_edge_after(const Tuple& loc) override;

    bool collapse_edge_before(const Tuple& loc) override
    {
        size_t v1_id = loc.vid(*this);

        if (m_vertex_attribute[v1_id].freeze) return false;

        auto loc1 = switch_vertex(loc);
        size_t v2_id = loc1.vid(*this);

        auto n1_locs = get_one_ring_tets_for_vertex(loc);
        auto n12_locs = get_incident_tets_for_edge(loc); // todo: duplicated computation

        std::map<size_t, double> qs;
        for (auto& l : n1_locs) {
            qs[l.tid(*this)] = m_tet_attribute[l.tid(*this)].quality; // get_quality(l);
        }
        for (auto& l : n12_locs) {
            qs.erase(l.tid(*this));
        }

        collapse_cache.local().max_energy = 0;
        for (auto& q : qs) {
            if (q.second > collapse_cache.local().max_energy)
                collapse_cache.local().max_energy = q.second;
            //
            collapse_cache.local().changed_tids.push_back(q.first);
        };
        return true;
    }

    bool collapse_edge_after(const Tuple& t) override
    {
        std::vector<Tuple> locs = get_one_ring_tets_for_vertex(t);

        for (auto& loc : locs) {
            if (is_inverted(loc)) {
                return false;
            }
        }

        for (size_t tid : collapse_cache.local().changed_tids) {
            auto tet = tuple_from_tet(tid);
            if (!tet.is_valid(*this)) continue;
            if (is_inverted(tet)) return false;
            double q = get_quality(tet);
            if (q > collapse_cache.local().max_energy) {
                return false;
            }
            m_tet_attribute[tid].quality = q;
        }
        return true;
    }

    struct SwapInfoCache
    {
        double max_energy;
    };
    tbb::enumerable_thread_specific<SwapInfoCache> swap_cache;
    bool swap_edge_before(const Tuple& t) override;
    bool swap_edge_after(const Tuple& t) override;
    bool swap_face_before(const Tuple& t) override;
    bool swap_face_after(const Tuple& t) override;
    bool swap_edge_44_before(const Tuple& t) override;
    bool swap_edge_44_after(const Tuple& t) override;
    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;
    //
    void split_all_edges();
    void smooth_all_vertices();
    void collapse_all_edges(bool is_limit_length = true);
    void swap_all_edges_44();
    void swap_all_edges();
    void swap_all_faces();
    //
    //
    size_t get_partition_id(const Tuple& loc) const
    {
        return m_vertex_attribute[loc.vid(*this)].partition_id;
    }
    std::tuple<double, double> get_max_avg_energy()
    {
        double max_energy = -1.;
        double avg_energy = 0;
        auto cnt = 0;

        TetMesh::for_each_tetra([&](auto& t) {
            auto& q = m_tet_attribute[t.tid(*this)].quality;
            if (q < 0) q = get_quality(t);

            if (q > max_energy) max_energy = q;

            avg_energy += std::cbrt(q);
            cnt++;
        });

        avg_energy /= cnt;

        return std::make_tuple(std::cbrt(max_energy), avg_energy);
    }
};
} // namespace app::interior_tet_opt
