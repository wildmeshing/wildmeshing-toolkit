#pragma once

#include <wmtk/Mesh.hpp>
#include <wmtk/io/ParaviewWriter.hpp>

namespace wmtk::components::wildmeshing::utils {

class IntermediateWrite
{
public:
    IntermediateWrite(
        const Mesh& mesh,
        const std::filesystem::path& out_dir,
        const std::filesystem::path& name,
        const std::string& vname,
        const bool intermediate_output);

    IntermediateWrite(IntermediateWrite&) = delete;

    void write();

    void disable_vertex_write();
    void disable_edge_write();
    void disable_face_write();
    void disable_tetrahedron_write();

private:
    const Mesh& m_mesh;
    const std::filesystem::path m_name;
    const std::string m_pos_attribute_name;
    const bool m_write;
    int64_t m_iter = 0;

    bool m_v = false;
    bool m_e = false;
    bool m_f = false;
    bool m_t = false;
};

} // namespace wmtk::components::wildmeshing::utils