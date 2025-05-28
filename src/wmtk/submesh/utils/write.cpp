#include "write.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/submesh/Embedding.hpp>
#include <wmtk/submesh/SubMesh.hpp>

void wmtk::submesh::utils::write(
    const std::filesystem::path& filename,
    const std::string& vertices_name,
    const SubMesh& sub,
    bool write_points,
    bool write_edges,
    bool write_faces,
    bool write_tetrahedra)
{
    const Mesh& m = sub.mesh();

    ParaviewWriter writer(
        filename,
        vertices_name,
        m,
        write_points,
        write_edges,
        write_faces,
        write_tetrahedra,
        [&sub](const simplex::IdSimplex& s) { return sub.contains(s); });
    m.serialize(writer);
}

void wmtk::submesh::utils::write(
    const std::filesystem::path& filename,
    const std::string& vertices_name,
    const Embedding& emb,
    bool write_points,
    bool write_edges,
    bool write_faces,
    bool write_tetrahedra)
{
    const Mesh& m = emb.mesh();

    ParaviewWriter writer(
        filename,
        vertices_name,
        m,
        write_points,
        write_edges,
        write_faces,
        write_tetrahedra);
    m.serialize(writer);
}
