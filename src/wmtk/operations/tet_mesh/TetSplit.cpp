#include "TetSplit.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/invariants/ValidTupleInvariant.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>
#include "EdgeCollapse.hpp"
#include "EdgeSplit.hpp"

namespace wmtk::operations {

void OperationSettings<tet_mesh::TetSplit>::create_invariants()
{
    split_settings.create_invariants();
    collapse_settings.create_invariants();

    invariants = std::make_shared<InvariantCollection>(m_mesh);
}

namespace tet_mesh {

TetSplit::TetSplit(TetMesh& m, const Simplex& t, const OperationSettings<TetSplit>& settings)
    : TetMeshOperation(m)
    , TupleOperation(settings.invariants, t)
    , m_settings{settings}
{
    assert(t.primitive_type() == PrimitiveType::Tetrahedron);
}

bool TetSplit::execute()
{
    Tuple first_split_ret;
    {
        EdgeSplit op(mesh(), input_simplex(), m_settings.split_settings);
        if (!op()) {
            return false;
        }
        first_split_ret = op.return_tuple();
    }


    Tuple second_split_ret;
    {
        EdgeSplit op(
            mesh(),
            Simplex::edge(mesh().switch_edge(mesh().switch_vertex(first_split_ret))),
            m_settings.split_settings);
        if (!op()) {
            return false;
        }
        second_split_ret = op.return_tuple();
    }

    Tuple third_split_ret;
    {
        EdgeSplit op(
            mesh(),
            Simplex::edge(
                mesh().switch_edge(mesh().switch_vertex(mesh().switch_face(second_split_ret)))),
            m_settings.split_settings);
        if (!op()) {
            return false;
        }
        third_split_ret = op.return_tuple();
    }

    Tuple first_collapse_ret;
    {
        EdgeCollapse op(
            mesh(),
            Simplex::edge(mesh().switch_face(mesh().switch_edge(third_split_ret))),
            m_settings.collapse_settings);
        if (!op()) {
            return false;
        }
        first_collapse_ret = op.return_tuple();
    }

    Tuple second_collapse_ret;
    {
        EdgeCollapse op(mesh(), Simplex::edge(first_collapse_ret), m_settings.collapse_settings);
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
    std::vector<Tuple> ret;
    if (type == PrimitiveType::Tetrahedron) {
        Simplex v(PrimitiveType::Vertex, mesh().switch_vertex(m_output_tuple));
        const SimplicialComplex& sc = SimplicialComplex::closed_star(mesh(), v);
        const wmtk::internal::SimplexSet& tets = sc.get_simplices(PrimitiveType::Tetrahedron);
        for (const Simplex& tet : tets) {
            ret.emplace_back(tet.tuple());
        }
        Simplex v1(PrimitiveType::Vertex, m_output_tuple);
        const SimplicialComplex& sc1 = SimplicialComplex::closed_star(mesh(), v1);
        const wmtk::internal::SimplexSet& tets1 = sc1.get_simplices(PrimitiveType::Tetrahedron);
        for (const Simplex& tet : tets1) {
            ret.emplace_back(tet.tuple());
        }
    } else if (type == PrimitiveType::Face) {
        Simplex v(PrimitiveType::Vertex, mesh().switch_vertex(m_output_tuple));
        const SimplicialComplex& sc = SimplicialComplex::closed_star(mesh(), v);
        const wmtk::internal::SimplexSet& faces = sc.get_simplices(PrimitiveType::Face);
        for (const Simplex& face : faces) {
            ret.emplace_back(face.tuple());
        }
        Simplex v1(PrimitiveType::Vertex, m_output_tuple);
        const SimplicialComplex& sc1 = SimplicialComplex::closed_star(mesh(), v1);
        const wmtk::internal::SimplexSet& faces1 = sc1.get_simplices(PrimitiveType::Face);
        for (const Simplex& face : faces1) {
            ret.emplace_back(face.tuple());
        }
    } else if (type == PrimitiveType::Edge) {
        Simplex v(PrimitiveType::Vertex, mesh().switch_vertex(m_output_tuple));
        const SimplicialComplex& sc = SimplicialComplex::closed_star(mesh(), v);
        const wmtk::internal::SimplexSet& edges = sc.get_simplices(PrimitiveType::Edge);
        for (const Simplex& edge : edges) {
            ret.emplace_back(edge.tuple());
        }
        Simplex v1(PrimitiveType::Vertex, m_output_tuple);
        const SimplicialComplex& sc1 = SimplicialComplex::closed_star(mesh(), v1);
        const wmtk::internal::SimplexSet& edges1 = sc1.get_simplices(PrimitiveType::Edge);
        for (const Simplex& edge : edges1) {
            ret.emplace_back(edge.tuple());
        }
    } else if (type == PrimitiveType::Vertex) {
        Simplex v(PrimitiveType::Vertex, mesh().switch_vertex(m_output_tuple));
        const SimplicialComplex& sc = SimplicialComplex::closed_star(mesh(), v);
        const wmtk::internal::SimplexSet& vertices = sc.get_simplices(PrimitiveType::Vertex);
        for (const Simplex& vertex : vertices) {
            ret.emplace_back(vertex.tuple());
        }
        Simplex v1(PrimitiveType::Vertex, m_output_tuple);
        const SimplicialComplex& sc1 = SimplicialComplex::closed_star(mesh(), v1);
        const wmtk::internal::SimplexSet& vertices1 = sc1.get_simplices(PrimitiveType::Vertex);
        for (const Simplex& vertex : vertices1) {
            ret.emplace_back(vertex.tuple());
        }
    }
    return ret;
}

} // namespace tet_mesh
} // namespace wmtk::operations
