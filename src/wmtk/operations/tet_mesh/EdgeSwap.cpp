#include "EdgeSwap.hpp"
#include <wmtk/TetMesh.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>

// #include <set>
#include "EdgeCollapse.hpp"
#include "EdgeSplit.hpp"

namespace wmtk::operations {
void OperationSettings<tet_mesh::EdgeSwap>::create_invariants()
{
    invariants = std::make_shared<InvariantCollection>(m_mesh);
    invariants->add(std::make_shared<InteriorEdgeInvariant>(m_mesh));

    collapse_settings.create_invariants();
    split_settings.create_invariants();
}

namespace tet_mesh {
EdgeSwap::EdgeSwap(
    Mesh& m,
    const Simplex& t,
    int collapse_index,
    const OperationSettings<EdgeSwap>& settings)
    : TetMeshOperation(m)
    , TupleOperation(settings.invariants, t)
    , m_collapse_index(collapse_index)
    , m_settings{settings}
{}

std::string EdgeSwap::name() const
{
    return "tet_mesh_edge_swap";
}

Tuple EdgeSwap::return_tuple() const
{
    return m_output_tuple;
}

std::vector<Tuple> EdgeSwap::new_tets_after_swap() const
{
    return m_new_tets;
}

std::vector<Tuple> EdgeSwap::new_edges_after_swap() const
{
    return m_new_edges;
}

bool EdgeSwap::execute()
{
    // Split the edge
    Tuple split_ret;

    tet_mesh::EdgeSplit split_op(mesh(), input_simplex(), m_settings.split_settings);
    if (!split_op()) {
        return false;
    }
    split_ret = split_op.return_tuple();


    // get candidate edges to collapse, which are the edges generated by split
    // the return tuple must be an interior edge, because we disallow swap on the boundary
    assert(!mesh().is_boundary_edge(split_ret));
    auto iter_tuple = split_ret;
    std::vector<Tuple> candidate_edges;
    while (true) {
        candidate_edges.push_back(mesh().switch_edge(iter_tuple));
        iter_tuple = mesh().switch_tetrahedron(mesh().switch_face(iter_tuple));
        if (iter_tuple == split_ret) {
            break;
        }
    }

    assert(m_collapse_index < candidate_edges.size());

    Tuple coll_ret;

    tet_mesh::EdgeCollapse coll_op(
        mesh(),
        Simplex::edge(candidate_edges[m_collapse_index]),
        m_settings.collapse_settings);
    if (!coll_op()) {
        return false;
    }


    // TODO: need to decide a convention for the output tuple
    m_output_tuple = coll_ret;
    // get new tets generated by swap = tets_generated_by_split - tets_deleted_by_collapse
    auto tet_ids_generated_by_split = split_op.new_tet_ids();
    auto tet_ids_deleted_by_collapse = coll_op.deleted_tet_ids();
    std::vector<long> tet_ids_generated_by_swap;
    std::set_difference(
        tet_ids_generated_by_split.begin(),
        tet_ids_generated_by_split.end(),
        tet_ids_deleted_by_collapse.begin(),
        tet_ids_deleted_by_collapse.end(),
        std::inserter(tet_ids_generated_by_swap, tet_ids_generated_by_swap.begin()));

    m_new_tets = get_tuples_from_ids(mesh(), PrimitiveType::Tetrahedron, tet_ids_generated_by_swap);

    // get new edges for modified primitives
    auto edge_ids_generated_by_split = split_op.new_edge_ids();
    auto edge_ids_deleted_by_collapse = coll_op.deleted_edge_ids();
    std::vector<long> edge_ids_generated_by_swap;
    std::set_difference(
        edge_ids_generated_by_split.begin(),
        edge_ids_generated_by_split.end(),
        edge_ids_deleted_by_collapse.begin(),
        edge_ids_deleted_by_collapse.end(),
        std::inserter(edge_ids_generated_by_swap, edge_ids_generated_by_swap.begin()));

    m_new_edges = get_tuples_from_ids(mesh(), PrimitiveType::Edge, edge_ids_generated_by_swap);

    return true;
}

std::vector<Simplex> EdgeSwap::modified_primitives() const
{
    // TODO: place holder, need to change
    const auto& edge_tuples = new_edges_after_swap();
    std::vector<Simplex> edges;
    edges.reserve(edge_tuples.size());
    std::transform(edge_tuples.begin(), edge_tuples.end(), edges.begin(), [](const Tuple& t) {
        return Simplex::edge(t);
    });
    return edges;
}

std::vector<Simplex> EdgeSwap::unmodified_primitives() const
{
    return {input_simplex()};
}


} // namespace tet_mesh


} // namespace wmtk::operations
