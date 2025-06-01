#include "EdgeCollapse.hpp"
#include <wmtk/utils/Logger.hpp>

#include <cassert>

#include <wmtk/EdgeMeshOperationExecutor.hpp>
#include <wmtk/TetMeshOperationExecutor.hpp>
#include <wmtk/TriMeshOperationExecutor.hpp>

// #include "utils/multi_mesh_edge_collapse.hpp"

#include <wmtk/submesh/Embedding.hpp>


namespace wmtk::operations {

EdgeCollapse::EdgeCollapse(Mesh& m)
    : Operation(m)
{
    // auto collect_attrs = [&](auto&& mesh) {
    //     // can have const variant values here so gotta filter those out
    //     if constexpr (!std::is_const_v<std::remove_reference_t<decltype(mesh)>>) {
    //         for (const auto& attr : mesh.custom_attributes()) {
    //             std::visit(
    //                 [&](auto&& tah) noexcept {
    //                     using HandleType = typename std::decay_t<decltype(tah)>;
    //                     if constexpr (attribute::MeshAttributeHandle::template
    //                     handle_type_is_basic<
    //                                       HandleType>()) {
    //                         using T = typename HandleType::Type;
    //                         m_new_attr_strategies.emplace_back(
    //                             std::make_shared<operations::CollapseNewAttributeStrategy<T>>(
    //                                 attribute::MeshAttributeHandle(mesh, attr)));
    //                     }
    //                 },
    //                 attr);
    //         }
    //     }
    // };

    // collect_attrs(m);
}

EdgeCollapse::EdgeCollapse(submesh::Embedding& m)
    : EdgeCollapse(m.mesh())
{
    m.set_collapse_strategies(*this);
}

std::vector<simplex::Simplex> EdgeCollapse::execute(const simplex::Simplex& simplex)
{
    switch (mesh().top_simplex_type()) {
    case PrimitiveType::Vertex: assert(false); break;
    case PrimitiveType::Edge: {
        EdgeMesh::EdgeMeshOperationExecutor exec(static_cast<EdgeMesh&>(mesh()), simplex.tuple());
        exec.collapse_edge();
        return {simplex::Simplex(PrimitiveType::Vertex, exec.m_output_tuple)};
    }
    case PrimitiveType::Triangle: {
        TriMesh::TriMeshOperationExecutor exec(static_cast<TriMesh&>(mesh()), simplex.tuple());
        exec.collapse_edge();
        return {simplex::Simplex(PrimitiveType::Vertex, exec.m_output_tuple)};
    }
    case PrimitiveType::Tetrahedron: {
        TetMesh::TetMeshOperationExecutor exec(static_cast<TetMesh&>(mesh()), simplex.tuple());
        exec.collapse_edge();
        return {simplex::Simplex(PrimitiveType::Vertex, exec.m_output_tuple)};
    }
    default: break;
    }

    return {};
}

std::vector<simplex::Simplex> EdgeCollapse::unmodified_primitives(
    const simplex::Simplex& simplex) const
{
    return mesh().parent_scope([&]() -> std::vector<simplex::Simplex> {
        const simplex::Simplex v0 = simplex::Simplex::vertex(mesh(), simplex.tuple());
        const simplex::Simplex v1 = simplex::Simplex::vertex(
            mesh(),
            mesh().switch_tuple(simplex.tuple(), PrimitiveType::Vertex));
        return {v0, v1};
    });
}
////////////////////////////////////

bool EdgeCollapse::after(
    const std::vector<simplex::Simplex>& unmods,
    const std::vector<simplex::Simplex>& mods) const
{
    if (mesh().is_free()) {
        return true;
    } else {
        return Operation::after(unmods, mods);
    }
}
} // namespace wmtk::operations
