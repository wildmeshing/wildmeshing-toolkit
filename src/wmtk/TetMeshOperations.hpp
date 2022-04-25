#include "TetMesh.h"

// Customizable Operations
namespace wmtk {
struct SplitEdge : public TetMesh::OperationBuilder
{
    const TetMesh& m;
    std::vector<size_t> affected;
    std::array<size_t, 2> edge_verts;
    size_t ux;

    SplitEdge(const TetMesh& _m)
        : m(_m)
    {}
    std::vector<size_t> removed_tids(const TetMesh::Tuple& t)
    {
        auto incidents = m.get_incident_tets_for_edge(t);
        for (auto i : incidents) {
            affected.push_back(i.tid(m));
        }
        edge_verts = {t.vid(m), t.switch_vertex(m).vid(m)};

        return affected;
    }
    int request_vert_slots() { return 1; }
    std::vector<std::array<size_t, 4>> replacing_tets(const std::vector<size_t>& slots)
    {
        assert(slots.size() == 1);
        ux = slots.front();

        auto new_tets = std::vector<std::array<size_t, 4>>();

        new_tets.reserve(2 * affected.size());
        for (auto i = 0; i < affected.size(); i++) {
            auto t_conn = m.oriented_tet_vids(m.tuple_from_tet(affected[i]));
            for (auto j = 0; j < 2; j++) {
                new_tets.push_back(t_conn);
                std::replace(new_tets.back().begin(), new_tets.back().end(),  edge_verts[j], ux);
            }
        }
        return new_tets;
    }
};
}