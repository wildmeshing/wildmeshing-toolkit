#include "EdgeSplit.hpp"

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/tet_mesh/EdgeOperationData.hpp>
#include <wmtk/submesh/Embedding.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/EdgeMeshOperationExecutor.hpp>
#include <wmtk/TetMeshOperationExecutor.hpp>
#include <wmtk/TriMeshOperationExecutor.hpp>

namespace wmtk::operations {

EdgeSplit::EdgeSplit(Mesh& m)
    : Operation(m)
{
    // auto collect_attrs = [&](auto&& mesh) {
    //     // can have const variant values here so gotta filter htose out
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
    //                             std::make_shared<operations::SplitNewAttributeStrategy<T>>(
    //                                 attribute::MeshAttributeHandle(mesh, attr)));
    //                     }
    //                 },
    //                 attr);
    //         }
    //     }
    // };

    // multimesh::MultiMeshVisitor custom_attribute_collector(collect_attrs);
    // custom_attribute_collector.execute_from_root(m);
}

EdgeSplit::EdgeSplit(submesh::Embedding& m)
    : EdgeSplit(m.mesh())
{
    m.set_split_strategies(*this);
}

std::vector<simplex::Simplex> EdgeSplit::execute(const simplex::Simplex& simplex)
{
    switch (mesh().top_simplex_type()) {
    case PrimitiveType::Vertex: assert(false); break;
    case PrimitiveType::Edge: {
        EdgeMesh::EdgeMeshOperationExecutor exec(static_cast<EdgeMesh&>(mesh()), simplex.tuple());
        exec.split_edge();
        return {simplex::Simplex(PrimitiveType::Vertex, exec.m_output_tuple)};
    }
    case PrimitiveType::Triangle: {
        TriMesh::TriMeshOperationExecutor exec(static_cast<TriMesh&>(mesh()), simplex.tuple());
        exec.split_edge();
        return {simplex::Simplex(PrimitiveType::Vertex, exec.m_output_tuple)};
    }
    case PrimitiveType::Tetrahedron: {
        TetMesh::TetMeshOperationExecutor exec(static_cast<TetMesh&>(mesh()), simplex.tuple());
        exec.split_edge();
        return {simplex::Simplex(PrimitiveType::Vertex, exec.m_output_tuple)};
    }
    default: break;
    }

    return {};
}
std::vector<simplex::Simplex> EdgeSplit::unmodified_primitives(
    const simplex::Simplex& simplex) const
{
    return {simplex};
}


std::pair<Tuple, Tuple> EdgeSplit::new_spine_edges(const Mesh& mesh, const Tuple& new_vertex)
{
    // new_vertex is a spine edge on a face pointing to the new vertex, so we
    // * PE -> new edge
    // * PF -> other face
    // * PE -> other spine edge
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Triangle;
    constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;

    std::pair<Tuple, Tuple> ret;

    switch (mesh.top_simplex_type()) {
    case PE: {
        ret = {new_vertex, mesh.switch_tuples(new_vertex, {PE})};
        break;
    }
    case PF: {
        ret = {new_vertex, mesh.switch_tuples(new_vertex, {PE, PF, PE})};
        break;
    }
    case PT: {
        ret = {new_vertex, mesh.switch_tuples(new_vertex, {PE, PF, PT, PF, PE})};
        break;
    }
    case PrimitiveType::Vertex:
    default: assert(false); // "Invalid top simplex"
    }
    return ret;
}


} // namespace wmtk::operations
