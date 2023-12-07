

#include "EdgeSplit.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/ValidTupleInvariant.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>
#include <wmtk/operations/utils/multi_mesh_edge_split.hpp>

namespace wmtk::operations {

std::shared_ptr<InvariantCollection> OperationSettings<tri_mesh::EdgeSplit>::create_invariants()
{
    std::make_shared<InvariantCollection> inv_col_ptr(m_mesh);
    if (!split_boundary_edges) {
        inv_col_ptr->add(std::make_shared<InteriorEdgeInvariant>(m));
    }
}

bool OperationSettings<tri_mesh::EdgeSplit>::are_invariants_initialized(
    std::shared_ptr<InvariantCollection> inv_col_ptr) const
{
    return find_invariants_in_collection_by_type<ValidTupleInvariant>(invariants);
}

namespace tri_mesh {

EdgeSplit::EdgeSplit(Mesh& m, const Tuple& t, const OperationSettings<EdgeSplit>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.invariants, t)
    , m_settings{settings}
{
    assert(m_settings.are_invariants_initialized());
}

bool EdgeSplit::execute()
{
    auto return_data = operations::utils::multi_mesh_edge_split(mesh(), input_tuple());

    const operations::tri_mesh::EdgeOperationData& my_data =
        return_data.get(mesh(), Simplex(PrimitiveType::Edge, input_tuple()));
    // move vertex to center of old vertices
    m_output_tuple = my_data.m_output_tuple;

    return true;
}

// potential after-like strucutre?
// bool EdgeSplit::after() const
//{
//    // energy decrease
//
//    //ConstAccessor<double> current_positions = {}; // current scope
//    //ConstAccessor<double> original_positions = {};// use checkpoint
//
//    //if(
//    //        get_area(current_positions, one_ring(m_output_tuple))
//    //        <
//    //        get_area(original_positions, one_ring(input_tuple()))) {
//    //    return false;;
//    //}
//}
std::string EdgeSplit::name() const
{
    return "tri_mesh_split_edge";
}

Tuple EdgeSplit::new_vertex() const
{
    return m_output_tuple;
}

Tuple EdgeSplit::return_tuple() const
{
    return m_output_tuple;
}

std::vector<Tuple> EdgeSplit::modified_primitives(PrimitiveType type) const
{
    if (type == PrimitiveType::Face) {
        // TODO
        // return modified_triangles();
    } else if (type == PrimitiveType::Vertex) {
        return {new_vertex()};
    }
    return {};
}
///std::vector<Tuple> EdgeSplit::triangle_onering() const
///{
///    Simplex v(PrimitiveType::Vertex, new_vertex());
///    auto sc = SimplicialComplex::open_star(v, mesh());
///    auto faces = sc.get_simplices(PrimitiveType::Face);
///    std::vector<Tuple> ret;
///    for (const auto& face : faces) {
///        ret.emplace_back(face.tuple());
///    }
///    return ret;
///}
///std::vector<Tuple> EdgeSplit::triangle_tworing() const
///{
///    throw "not implemented";
///    return {};
///}

// std::vector<Tuple> new_triangles() const  {
//     return triangle_onering();
// }
// std::vector<Tuple> new_edges() const  {
//     return spline_edges + rib_edges();
// }


// bool MySplitEdge::after()const {
//
//    //for(const auto& tri: modified_triangles()) {
//    //    positive_area_invariant(m,tri);
//    //}
//
//
//}
} // namespace tri_mesh
} // namespace wmtk::operations
