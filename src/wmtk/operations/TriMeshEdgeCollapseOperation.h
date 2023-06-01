#pragma once
#include <wmtk/operations/SingleTupleTriMeshOperation.h>


namespace wmtk {

/**
 * Collapse an edge
 *
 * @param t Input Tuple for the edge to be collapsed.
 * @param[out] new_edges a vector of Tuples refering to the triangles incident to the new vertex
 * introduced
 * @note collapse edge a,b and generate a new vertex c
 * @return if collapse succeed
 *
 *
 *
 */
// Given an input triangle with tuple X
//   --------------------------
//   |\          /\          /|
//   | \        /  \   A    / |
//   |  \      /    \      /  |
//   |   \    /  X   \    /   |
//   |    \  /        \  /    |
//   |     \/          \/     |
//   ------X-----X-------------
//   |     /\          /\     |
//   |    /  \        /  \    |
//   |   /    \      /    \   |
//   |  /      \    /      \  |
//   | /        \  /        \ |
//   |/          \/          \|
//   --------------------------
//
// it is transformed to
//   ---------------
//   |\     |     /|
//   | \ o  | X  / |
//   |  \   o   /  |
//   |   \  |  X   |
//   |    \ | /    |
//   |     \|/     |
//   -------X-------
//   |     /|\     |
//   |    / | \    |
//   |   /  |  \   |
//   |  /   |   \  |
//   | /    |    \ |
//   |/     |     \|
//   ---------------
//   returns X, or if the triangle A does not exist then edge/face O are chosen
class LinksOfVertex;
class TriMeshEdgeCollapseOperation : public SingleTupleTriMeshOperation
{
public:
    bool execute(TriMesh& m, const Tuple& t) override;
    bool before(TriMesh& m, const Tuple& t) override;
    bool after(TriMesh& m) override;
    std::string name() const override;

    /**
     * @brief prerequisite for collapse
     * @param t Tuple referes to the edge to be collapsed
     * @returns true is the link check is passed
     */
    static bool check_link_condition(const TriMesh& m, const Tuple& t);


    std::vector<Tuple> modified_triangles(const TriMesh& m) const override;
    std::optional<Tuple> new_vertex(const TriMesh& m) const;

    // protected:
    constexpr static size_t link_dummy = std::numeric_limits<size_t>::max();
    std::vector<size_t> fids_containing_edge(const TriMesh& m, const Tuple& t) const;

    static LinksOfVertex links_of_vertex(const TriMesh& m, const Tuple& vertex);


    // returns the vids opposite to an edge, including a bool for the "infinite" vertex if no
    // opposite exists
    static std::tuple<std::vector<size_t>, bool> edge_link_of_edge_vids(
        const TriMesh& m,
        const Tuple& edge);
    // static std::vector<Tuple> edge_link_of_edge(const TriMesh& m, const Tuple& edge);
};


struct LinksOfVertex
{
    std::vector<size_t> vertex;
    bool infinite_vertex = false;

    // finite edges as vids on the opposite edge of each triangle
    std::vector<std::array<size_t, 2>> edge;

    std::vector<std::array<size_t, 2>> edge_test;

    // vids of boundary edges (infinite link edges)
    std::vector<size_t> infinite_edge;
};
} // namespace wmtk
