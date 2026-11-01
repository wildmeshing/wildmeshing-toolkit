#pragma once
#include <wmtk/TetMesh.h>
#include <type_traits>
#include <wmtk/io/tagged_mesh_data.hpp>

namespace wmtk::components::topological_offset {
template <typename Mesh>
void collect_memory_output(
    Mesh& mesh,
    io::ComponentIO& context,
    io::OutputKeys keys = io::default_output_keys())
{
    // In 2D, `_surf` is the envelope curve itself, which is returned as `envelope` below.
    if constexpr (!std::is_base_of_v<TetMesh, Mesh>) keys["mesh_surf.vtu"] = "";
    MeshMap raw;
    mesh.write_vtu("mesh", &raw);
    // The stencil file (3D, DEBUG_stencil, a mesh with an offset potential and a front): one point
    // per stencil point of every offset face, indexed by those points, not by the mesh's vertex
    // slots, so compact_slot_outputs() cannot remap it ("Export references an inactive vertex":
    // every run of the 2026-10-29 sonic sweep aborted there). A debug-frame diagnostic, not a
    // result.
    raw.erase("mesh_stencil.vtu");
    if constexpr (std::is_base_of_v<TetMesh, Mesh>) io::compact_slot_outputs(mesh, raw);
    context.result.first = io::named_outputs(std::move(raw), keys);
    auto& data = context.result.first.at("mesh");
    if constexpr (std::is_base_of_v<TetMesh, Mesh>) {
        io::collect_tags(
            mesh,
            context.result.first,
            CellType::Tetrahedron,
            mesh.get_tets(),
            [&](const auto& t) -> const auto& { return mesh.m_tet_attribute[t.tid(mesh)].tag; });
        io::collect_face_tags(mesh, context.result.first, mesh.m_face_tag_names);
    } else {
        io::collect_tags(
            mesh,
            context.result.first,
            CellType::Triangle,
            mesh.get_faces(),
            [&](const auto& t) -> const auto& { return mesh.m_face_attribute[t.fid(mesh)].tags; });
    }
    if (mesh.m_offset_params.save_offset_correspondence) {
        const auto ids = data.vertex_attributes.at("vid").integers();
        DataMatrix<int64_t> correspondence(data.V.rows(), 1);
        for (Eigen::Index i = 0; i < ids.rows(); ++i)
            correspondence(i, 0) = mesh.m_vertex_extra[ids(i, 0)].m_corr_input_vid;
        data.vertex_attributes["corr_input_vid"].values = std::move(correspondence);
        data.vertex_attributes["vid"].values = ids;
    }
    if (mesh.m_has_envelope) {
        context.result.first["envelope"] = io::geometry_data(
            mesh.m_V_envelope,
            mesh.m_F_envelope,
            std::is_base_of_v<TetMesh, Mesh> ? CellType::Triangle : CellType::Edge);
    }
    data.validate();
}
} // namespace wmtk::components::topological_offset
