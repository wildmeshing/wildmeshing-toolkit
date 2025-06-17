#pragma once

#include <filesystem>
#include <paraviewo/VTUWriter.hpp>
#include <wmtk/Types.hpp>

namespace wmtk::io {

template <typename MeshT>
class VTUWriter
{
    using Tuple = typename MeshT::Tuple;

public:
    VTUWriter(MeshT& mesh);

    bool write_triangles(const std::filesystem::path& filename)
    {
        Eigen::MatrixXi F;
        {
            const std::vector<Tuple> face_tuples = m_mesh.get_faces();

            if (face_tuples.empty()) {
                return false; // cannot print mesh without faces
            }

            F.resize(face_tuples.size(), 3);
            for (int i = 0; i < face_tuples.size(); ++i) {
                const auto vs = m_mesh.oriented_tri_vids(face_tuples[i]);
                for (int j = 0; j < 3; j++) {
                    F(i, j) = vs[j];
                }
            }
        }
        Eigen::MatrixXd V;
        std::vector<MatrixXd> VA; // vertex attributes
        std::vector<std::string> VA_names;
        {
            const std::vector<Tuple> vertex_tuples = m_mesh.get_vertices();

            if (vertex_tuples.empty()) {
                return false; // cannot print mesh without vertices
            }

            // init VA
            VA_names = m_mesh.serialize_vertex_attributes_names();
            {
                VA.resize(VA_names.size());
                const std::vector<VectorXd> v0_attrs = m_mesh.serialize_vertex_attributes(0);
                assert(VA_names.size() == v0_attrs.size());
                for (int i = 0; i < v0_attrs.size(); ++i) {
                    VA[i].resize(vertex_tuples.size(), v0_attrs[i].size());
                }
            }

            // init V
            V.resize(vertex_tuples.size(), m_mesh.position(0).size());

            for (const Tuple& v : vertex_tuples) {
                const auto vid = v.vid(m_mesh);
                V.row(vid) = m_mesh.position(vid);

                const std::vector<VectorXd> attrs = m_mesh.serialize_vertex_attributes(vid);
                for (int i = 0; i < attrs.size(); ++i) {
                    VA[i].row(vid) = attrs[i];
                }
            }
        }

        paraviewo::VTUWriter writer;
        for (int i = 0; i < VA.size(); ++i) {
            writer.add_field(VA_names[i], VA[i]);
        }
        bool r = writer.write_mesh(filename.string(), V, F);
        return r;
    }

private:
    MeshT& m_mesh;
};

template <typename MeshT>
inline VTUWriter<MeshT>::VTUWriter(MeshT& mesh)
    : m_mesh(mesh)
{}

} // namespace wmtk::io
