#include "TetSplit.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/invariants/ValidTupleInvariant.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>
#include "EdgeCollapse.hpp"
#include "EdgeSplit.hpp"

namespace wmtk::operations {

void OperationSettings<tet_mesh::TetSplit>::initialize_invariants(const TetMesh& m)
{
    // outdated + is valid tuple
    invariants = basic_invariant_collection(m);
}

bool OperationSettings<tet_mesh::TetSplit>::are_invariants_initialized() const
{
    return find_invariants_in_collection_by_type<ValidTupleInvariant>(invariants);
}

namespace tet_mesh {

TetSplit::TetSplit(TetMesh& m, const Tuple& t, const OperationSettings<TetSplit>& settings)
    : TetMeshOperation(m)
    , TupleOperation(settings.invariants, t)
    , m_settings{settings}
{
    assert(m_settings.are_invariants_initialized());
}

bool TetSplit::execute()
{
    Tuple first_split_ret;
    {
        OperationSettings<EdgeSplit> settings;
        settings.initialize_invariants(mesh());
        EdgeSplit op(mesh(), input_tuple(), settings);
        if (!op()) {
            return false;
        }
        first_split_ret = op.return_tuple();
    }


    Tuple second_split_ret;
    {
        OperationSettings<EdgeSplit> settings;
        settings.initialize_invariants(mesh());
        EdgeSplit op(mesh(), mesh().switch_edge(mesh().switch_vertex(first_split_ret)), settings);
        if (!op()) {
            return false;
        }
        second_split_ret = op.return_tuple();
    }

    Tuple third_split_ret;
    {
        OperationSettings<EdgeSplit> settings;
        settings.initialize_invariants(mesh());
        EdgeSplit op(
            mesh(),
            mesh().switch_edge(mesh().switch_vertex(mesh().switch_face(second_split_ret))),
            settings);
        if (!op()) {
            return false;
        }
        third_split_ret = op.return_tuple();
    }

    Tuple first_collapse_ret;
    {
        OperationSettings<EdgeCollapse> settings;
        settings.initialize_invariants(mesh());
        EdgeCollapse op(mesh(), mesh().switch_face(mesh().switch_edge(third_split_ret)), settings);
        if (!op()) {
            return false;
        }
        first_collapse_ret = op.return_tuple();
    }

    Tuple second_collapse_ret;
    {
        OperationSettings<EdgeCollapse> settings;
        settings.initialize_invariants(mesh());
        EdgeCollapse op(mesh(), first_collapse_ret, settings);
        if (!op()) {
            return false;
        }
        second_collapse_ret = op.return_tuple();
    }

    m_output_tuple = mesh().switch_face(mesh().switch_tetrahedron(second_collapse_ret));

    return true;
}

std::string TetSplit::name() const
{
    return "tet_mesh_split_tet";
}

Tuple TetSplit::new_vertex() const
{
    return m_output_tuple;
}

Tuple TetSplit::return_tuple() const
{
    return m_output_tuple;
}

std::vector<Tuple> TetSplit::modified_primitives(PrimitiveType type) const
{
    Simplex v(PrimitiveType::Vertex, mesh().switch_vertex(m_output_tuple));
    const SimplicialComplex& sc = SimplicialComplex::closed_star(mesh(), v);
    std::vector<Tuple> ret;
    if (type == PrimitiveType::Tetrahedron) {
        const wmtk::internal::SimplexSet& tets = sc.get_simplices(PrimitiveType::Tetrahedron);
        if (!mesh().is_boundary(mesh().switch_face(
                mesh().switch_edge(mesh().switch_tetrahedron(m_output_tuple))))) {
            for (const Simplex& tet : tets) {
                ret.emplace_back(tet.tuple());
            }
            ret.emplace_back(mesh().switch_tetrahedron(
                mesh().switch_face(mesh().switch_edge(mesh().switch_tetrahedron(m_output_tuple)))));
        }
    } else if (type == PrimitiveType::Face) {
        const wmtk::internal::SimplexSet& faces = sc.get_simplices(PrimitiveType::Face);
        for (const Simplex& face : faces) {
            ret.emplace_back(face.tuple());
        }
        if (!mesh().is_boundary(mesh().switch_face(
                mesh().switch_edge(mesh().switch_tetrahedron(m_output_tuple))))) {
            Tuple low_t = mesh().switch_tetrahedron(
                mesh().switch_face(mesh().switch_edge(mesh().switch_tetrahedron(m_output_tuple))));
            ret.emplace_back(mesh().switch_face(low_t));
            ret.emplace_back(mesh().switch_face(mesh().switch_edge(low_t)));
            ret.emplace_back(mesh().switch_face(mesh().switch_edge(mesh().switch_vertex(low_t))));
        }
    } else if (type == PrimitiveType::Edge) {
        const wmtk::internal::SimplexSet& edges = sc.get_simplices(PrimitiveType::Edge);
        for (const Simplex& edge : edges) {
            ret.emplace_back(edge.tuple());
        }
        if (!mesh().is_boundary(mesh().switch_face(
                mesh().switch_edge(mesh().switch_tetrahedron(m_output_tuple))))) {
            Tuple low_t = mesh().switch_tetrahedron(
                mesh().switch_face(mesh().switch_edge(mesh().switch_tetrahedron(m_output_tuple))));
            ret.emplace_back(low_t);
            ret.emplace_back(mesh().switch_edge(low_t));
            ret.emplace_back(mesh().switch_edge(mesh().switch_vertex(low_t)));
            ret.emplace_back(mesh().switch_edge(mesh().switch_face(low_t)));
            ret.emplace_back(mesh().switch_edge(mesh().switch_face(mesh().switch_edge(low_t))));
            ret.emplace_back(mesh().switch_edge(
                mesh().switch_face(mesh().switch_vertex(mesh().switch_edge(low_t)))));
        }
    } else if (type == PrimitiveType::Vertex) {
        const wmtk::internal::SimplexSet& vertices = sc.get_simplices(PrimitiveType::Vertex);
        for (const Simplex& vertex : vertices) {
            ret.emplace_back(vertex.tuple());
        }
        if (!mesh().is_boundary(mesh().switch_face(
                mesh().switch_edge(mesh().switch_tetrahedron(m_output_tuple))))) {
            Tuple low_t = mesh().switch_tetrahedron(
                mesh().switch_face(mesh().switch_edge(mesh().switch_tetrahedron(m_output_tuple))));
            ret.emplace_back(low_t);
            std::vector<Simplex> lower_tuples = SimplicialComplex::k_ring(mesh(), low_t, 1);
            for (const Simplex& vertex : vertices) {
                ret.emplace_back(vertex.tuple());
            }
        }
    }
    return ret;
}

} // namespace tet_mesh
} // namespace wmtk::operations
