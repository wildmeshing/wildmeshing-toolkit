#pragma once
#include <wmtk/TriMeshOperation.h>


namespace wmtk {
struct LinksOfVertex
{
    std::vector<size_t> vertex;
    std::vector<std::pair<size_t, size_t>> edge;
};
/**
 * Collapse an edge
 *
 * @param t Input Tuple for the edge to be collapsed.
 * @param[out] new_edges a vector of Tuples refering to the triangles incident to the new vertex
 * introduced
 * @note collapse edge a,b and generate a new vertex c
 * @return if collapse succeed
 */
class TriMeshEdgeCollapseOperation : public TriMeshOperation
{
public:
    ExecuteReturnData execute(TriMesh& m, const Tuple& t) override;
    bool before(TriMesh& m, const Tuple& t) override;
    bool after(TriMesh& m, ExecuteReturnData& ret_data) override;
    std::string name() const override;

    /**
     * @brief prerequisite for collapse
     * @param t Tuple referes to the edge to be collapsed
     * @returns true is the link check is passed
     */
    static bool check_link_condition(const TriMesh& m, const Tuple& t);

    // computes the 
    static LinksOfVertex links_of_vertex(const TriMesh& m, const Tuple& vertex);
    static std::vector<size_t> edge_link_of_edge(const TriMesh& m, const Tuple& edge);

};
}
