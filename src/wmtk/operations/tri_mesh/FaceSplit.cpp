#include "FaceSplit.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/ValidTupleInvariant.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>
#include "EdgeCollapse.hpp"
#include "EdgeSplit.hpp"

namespace wmtk::operations {
void OperationSettings<tri_mesh::FaceSplit>::create_invariants()
{
    split_settings.create_invariants();
    collapse_settings.create_invariants();

    // invariants = basic_invariant_collection(m);
    invariants = std::make_shared<InvariantCollection>(m_mesh);
}

namespace tri_mesh {

FaceSplit::FaceSplit(Mesh& m, const Simplex& t, const OperationSettings<FaceSplit>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.invariants, t)
    , m_settings{settings}
{}

std::string FaceSplit::name() const
{
    return "tri_mesh_face_split";
}

bool FaceSplit::before() const
{
    return TupleOperation::before();
}

Tuple FaceSplit::return_tuple() const
{
    return m_output_tuple;
}

std::vector<Tuple> FaceSplit::modified_primitives(PrimitiveType type) const
{
    Simplex v(PrimitiveType::Vertex, m_output_tuple);
    auto sc = SimplicialComplex::open_star(mesh(), v);
    std::vector<Tuple> ret;
    if (type == PrimitiveType::Face) {
        auto faces = sc.get_simplices(PrimitiveType::Face);
        for (const auto& face : faces) {
            ret.emplace_back(face.tuple());
        }
    } else if (type == PrimitiveType::Edge) {
        auto edges = sc.get_simplices(PrimitiveType::Edge);
        for (const auto& edge : edges) {
            ret.emplace_back(edge.tuple());
        }
    } else if (type == PrimitiveType::Vertex) {
        auto vertices = sc.get_simplices(PrimitiveType::Vertex);
        for (const auto& vertex : vertices) {
            ret.emplace_back(vertex.tuple());
        }
    }
    return ret;
}

bool FaceSplit::execute()
{
    // input
    //     p1
    //    / \ .
    //   / f \ .
    //  /     \ .
    // p0-->--p2
    //  \     /
    //   \   /
    //    \ /

    Tuple split_ret;
    {
        tri_mesh::EdgeSplit split_op(mesh(), input_simplex(), m_settings.split_settings);
        if (!split_op()) {
            return false;
        }
        split_ret = split_op.return_tuple();
    }

    // after split
    //    /|\ .
    //   / | \ .
    //  /  | f\ .
    //  ---X-->
    //  \  |  /
    //   \ | /
    //    \|/

    // switch edge - switch face
    //    /|\ .
    //   / v \ .
    //  /f |  \ .
    //  ---X---
    //  \  |  /
    //   \ | /
    //    \|/
    const Tuple second_split_input_tuple = mesh().switch_vertex(mesh().switch_edge(split_ret));
    Tuple second_split_ret;
    {
        tri_mesh::EdgeSplit split_op(
            mesh(),
            Simplex::edge(second_split_input_tuple),
            m_settings.split_settings);
        if (!split_op()) {
            return false;
        }
        second_split_ret = split_op.return_tuple();
    }
    // after split
    //
    //     /|\ .
    //    / | \ .
    //   /  X  \ .
    //  /  /|\  \ .
    // /__/_v_\__\ .
    //  \   |   /
    //   \  |  /
    //    \ | /
    //     \|/

    // collapse the split ret
    //     /|\ .
    //    / | \ .
    //   / /|\ \ .
    //  / / | \ \ .
    //  |/__X__\>
    //  \   |   /
    //   \  |  /
    //    \ | /
    //     \|/
    Tuple coll_ret;
    {
        const Tuple coll_input_tuple = mesh().switch_edge(mesh().switch_vertex(second_split_ret));
        tri_mesh::EdgeCollapse coll_op(
            mesh(),
            Simplex::edge(coll_input_tuple),
            m_settings.collapse_settings);
        if (!coll_op()) {
            return false;
        }
        coll_ret = coll_op.return_tuple();
    }
    // collapse output
    //     /| \ .
    //    / |  \ .
    //   /  *   \ .
    //  / /   \  \ .
    // / /  f   > \ .
    // |/_ _ _ _ \|
    //  \        /
    //   \      /
    //    \    /
    //     \  /
    // return new vertex's tuple
    m_output_tuple = mesh().switch_edge(mesh().switch_vertex(coll_ret));
    return true;
}


} // namespace tri_mesh
} // namespace wmtk::operations
