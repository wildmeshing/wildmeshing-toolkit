#include "IntermediateWrite.hpp"

#include <wmtk/utils/Logger.hpp>

wmtk::components::wildmeshing::utils::IntermediateWrite::IntermediateWrite(
    const Mesh& mesh,
    const std::filesystem::path& out_dir,
    const std::filesystem::path& name,
    const std::string& vname,
    const bool intermediate_output)
    : m_mesh(mesh)
    , m_name(out_dir / name)
    , m_pos_attribute_name(vname)
    , m_write(intermediate_output)
{
    switch (m_mesh.top_simplex_type()) {
    case PrimitiveType::Tetrahedron: m_t = true; [[fallthrough]];
    case PrimitiveType::Triangle: m_f = true; [[fallthrough]];
    case PrimitiveType::Edge: m_e = true; [[fallthrough]];
    case PrimitiveType::Vertex: m_v = true; break;
    default: log_and_throw_error("Unkown primitive type in IntermediateWrite");
    }
}

void wmtk::components::wildmeshing::utils::IntermediateWrite::write()
{
    if (!m_write) {
        return;
    }
    wmtk::io::ParaviewWriter writer(
        fmt::format("{}_{}", m_name.string(), m_iter++),
        m_pos_attribute_name,
        m_mesh,
        m_v,
        m_e,
        m_f,
        m_t);
    m_mesh.serialize(writer);
}

void wmtk::components::wildmeshing::utils::IntermediateWrite::disable_vertex_write()
{
    m_v = false;
}

void wmtk::components::wildmeshing::utils::IntermediateWrite::disable_edge_write()
{
    m_e = false;
}

void wmtk::components::wildmeshing::utils::IntermediateWrite::disable_face_write()
{
    m_f = false;
}

void wmtk::components::wildmeshing::utils::IntermediateWrite::disable_tetrahedron_write()
{
    m_t = false;
}
