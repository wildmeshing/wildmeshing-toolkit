

#include "EdgeSplit.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/ValidTupleInvariant.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

namespace wmtk::operations {

void OperationSettings<tri_mesh::EdgeSplit>::initialize_invariants(const TriMesh& m)
{
    // outdated + is valid tuple
    invariants = basic_invariant_collection(m);

    if (!split_boundary_edges) {
        invariants.add(std::make_shared<InteriorEdgeInvariant>(m));
    }
}

bool OperationSettings<tri_mesh::EdgeSplit>::are_invariants_initialized() const
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
    // move vertex to center of old vertices
    m_output_tuple = mesh().split_edge(input_tuple(), hash_accessor());

    //    for(const acc: tri_accessors) {
    //    ConstACcessor old_tri_acc(acc, checkpoint);
    // for(tri: new_triangles) {

    //    value = old_tri_acc(input_tuple());
    //        acc.assign(old,tri);
    //    }
    //}
    // for(edge: new_edges) {
    //    for(const acc: edge_accessors) {
    //        acc.assign(old,edge);
    //    }
    //}
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
