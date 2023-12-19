

#include "EdgeSplit.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>
#include <wmtk/operations/utils/multi_mesh_edge_split.hpp>

namespace wmtk::operations {

void OperationSettings<tri_mesh::EdgeSplit>::create_invariants()
{
    invariants = std::make_shared<InvariantCollection>(m_mesh);
    if (!split_boundary_edges) {
        invariants->add(std::make_shared<InteriorEdgeInvariant>(m_mesh));
    }
}

namespace tri_mesh {

EdgeSplit::EdgeSplit(Mesh& m, const Simplex& t, const OperationSettings<EdgeSplit>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.invariants, t)
    , m_settings{settings}
{
    assert(t.primitive_type() == PrimitiveType::Edge);
    assert(m_settings.invariants);
}

bool EdgeSplit::execute()
{
    auto return_data = operations::utils::multi_mesh_edge_split(mesh(), input_tuple());

    spdlog::warn("{}", primitive_type_name(input_simplex().primitive_type()));

    const operations::tri_mesh::EdgeOperationData& my_data =
        return_data.get(mesh(), input_simplex());
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
std::array<Tuple, 2> EdgeSplit::new_spine_edges() const
{
    // m_output_tuple is a spine edge on a face pointing to the new vertex, so we
    // * PE -> new edge
    // * PF -> other face
    // * PE -> other spine edge
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Face;
    std::array<Tuple, 2> r{{new_vertex(), mesh().switch_tuples(new_vertex(), {PE, PF, PE})}};
    return r;
}

Tuple EdgeSplit::return_tuple() const
{
    return m_output_tuple;
}

std::vector<Simplex> EdgeSplit::modified_primitives() const
{
    std::vector<Simplex> s;
    s.reserve(3);
    s.emplace_back(simplex::Simplex::vertex(new_vertex()));

    for (const auto& et : new_spine_edges()) {
        s.emplace_back(simplex::Simplex::edge(et));
    }
    return s;
}

std::vector<Simplex> EdgeSplit::unmodified_primitives() const
{
    throw std::runtime_error("not implemented");
    return std::vector<Simplex>();
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
