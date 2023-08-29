
#include "TriMeshSplitEdgeOperation.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/ValidTupleInvariant.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>
#include "wmtk/SimplicialComplex.hpp"

namespace wmtk {

void OperationSettings<TriMeshSplitEdgeOperation>::initialize_invariants(const TriMesh& m)
{
    // outdated + is valid tuple
    invariants = basic_invariant_collection(m);

    if (!split_boundary_edges) {
        invariants.add(std::make_shared<InteriorEdgeInvariant>(m));
    }
}

bool OperationSettings<TriMeshSplitEdgeOperation>::are_invariants_initialized() const
{
    return find_invariants_in_collection_by_type<ValidTupleInvariant>(invariants);
}


TriMeshSplitEdgeOperation::TriMeshSplitEdgeOperation(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<TriMeshSplitEdgeOperation>& settings)
    : TupleOperation(m, settings.invariants, t)
    , m_settings{settings}
{
    assert(m_settings.are_invariants_initialized());
}

bool TriMeshSplitEdgeOperation::execute()
{
    // move vertex to center of old vertices
    TriMesh& m = dynamic_cast<TriMesh&>(m_mesh);
    m_output_tuple = m.split_edge(input_tuple());

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
// bool TriMeshSplitEdgeOperation::after() const
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
std::string TriMeshSplitEdgeOperation::name() const
{
    return "tri_mesh_split_edge";
}

Tuple TriMeshSplitEdgeOperation::new_vertex() const
{
    return m_output_tuple;
}

Tuple TriMeshSplitEdgeOperation::return_tuple() const
{
    return m_output_tuple;
}

std::vector<Tuple> TriMeshSplitEdgeOperation::modified_primitives(PrimitiveType type) const
{
    if (type == PrimitiveType::Face) {
        // TODO
        // return modified_triangles();
    } else if (type == PrimitiveType::Vertex) {
        return {new_vertex()};
    }
    return {};
}
///std::vector<Tuple> TriMeshSplitEdgeOperation::triangle_onering() const
///{
///    Simplex v(PrimitiveType::Vertex, new_vertex());
///    auto sc = SimplicialComplex::open_star(v, m_mesh);
///    auto faces = sc.get_simplices(PrimitiveType::Face);
///    std::vector<Tuple> ret;
///    for (const auto& face : faces) {
///        ret.emplace_back(face.tuple());
///    }
///    return ret;
///}
///std::vector<Tuple> TriMeshSplitEdgeOperation::triangle_tworing() const
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
} // namespace wmtk
