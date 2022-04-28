#pragma once

#include <wmtk/ConcurrentTetMesh.h>
#include <Eigen/Core>

namespace interior_tetopt {
struct VertexAttribute
{
    bool freeze = false;
    Eigen::Vector3d pos;
};

struct TetraAttribute
{
    double quality;
};


struct InteriorTetOpt : wmtk::ConcurrentTetMesh
{
    InteriorTetOpt() {}
    ~InteriorTetOpt() {}

    using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    using TetAttCol = wmtk::AttributeCollection<TetAttributes>;
    VertAttCol m_vertex_attribute;
    TetAttCol m_tet_attribute;

    struct SplitInfoCache
    {
        size_t v1_id;
        size_t v2_id;
    };
    tbb::enumerable_thread_specific<SplitInfoCache> split_cache;
    bool split_edge_before(const Tuple& loc0) override
    {
        if (is_edge_on_surface(loc0)) return false;

        split_cache.local().v1_id = loc0.vid(*this);
        auto loc1 = loc0.switch_vertex(*this);
        split_cache.local().v2_id = loc1.vid(*this);
        //
        size_t v1_id = split_cache.local().v1_id;
        size_t v2_id = split_cache.local().v2_id;
    }
    bool split_edge_after(const Tuple& loc) override
    {
        std::vector<Tuple> locs = get_one_ring_tets_for_vertex(loc);
        auto vx = loc.vid(*this);

        auto v1 = split_cache.local().v1_id;
        auto v2 = split_cache.local().v2_id;
        m_vertex_attribute[vx].pos = (m_vertex_attribute[v1] + m_vertex_attribute[v2]) / 2;
        for (auto& loc : locs) {
            if (is_inverted(loc)) {
                return false;
            }
        }
        for (auto& loc : locs) {
            m_tet_attribute[loc.tid(*this)].quality = get_quality(loc);
        }
    }

    bool collapse_edge_before(const Tuple& loc) override
    {
        size_t v1_id = loc.vid(*this);
        auto loc1 = switch_vertex(loc);
        size_t v2_id = loc1.vid(*this);
        if (m_vertex_attribute[v1_id].freeze) return false;

        auto n1_locs = get_one_ring_tets_for_vertex(loc);
        auto n12_locs = get_incident_tets_for_edge(loc); // todo: duplicated computation

        std::map<size_t, double> qs;
        for (auto& l : n1_locs) {
            qs[l.tid(*this)] = m_tet_attribute[l.tid(*this)].m_quality; // get_quality(l);
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

        std::vector<double> qs;
        for (size_t tid : collapse_cache.local().changed_tids) {
            auto tet = tuple_from_tet(tid);
            if (is_inverted(tet)) return false;
            double q = get_quality(tet);
            if (q > collapse_cache.local().max_energy) {
                return false;
            }
            qs.push_back(q);
        }
        for (int i = 0; i < collapse_cache.local().changed_tids.size(); i++) {
            m_tet_attribute[collapse_cache.local().changed_tids[i]].m_quality = qs[i];
        }
        return true;
    }

    bool swap_edge_before(const Tuple& t) override;
    bool swap_edge_after(const Tuple& t) override;
    bool swap_face_before(const Tuple& t) override;
    bool swap_face_after(const Tuple& t) override;
    bool swap_edge_44_before(const Tuple& t) override;
    bool swap_edge_44_after(const Tuple& t) override;
};
} // namespace interior_tetopt
