#pragma once
#include <wmtk/TriMesh.h>
#include <paraviewo/VTUWriter.hpp>

class TriMeshWithWriter : public wmtk::TriMesh
{
    struct VertexAttributes
    {
        Eigen::Vector2d pos;
        size_t idx;
    };

    wmtk::AttributeCollection<VertexAttributes> vertex_attrs;

public:
    void create_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
    {
        // Register attributes
        p_vertex_attrs = &vertex_attrs;

        // Convert from eigen to internal representation
        std::vector<std::array<size_t, 3>> tri(F.rows());

        for (int i = 0; i < F.rows(); i++) {
            for (int j = 0; j < 3; j++) {
                tri[i][j] = (size_t)F(i, j);
            }
        }
        // Initialize the trimesh class which handles connectivity
        wmtk::TriMesh::create_mesh(V.rows(), tri);
        // Save the vertex position in the vertex attributes
        for (unsigned i = 0; i < V.rows(); ++i) {
            vertex_attrs[i].pos << V.row(i)[0], V.row(i)[1];
            vertex_attrs[i].idx = i;
        }
    }

    // Exports V and F of the stored mesh
    void export_mesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F) const
    {
        V = Eigen::MatrixXd::Zero(vert_capacity(), 2);
        for (auto& t : get_vertices()) {
            auto i = t.vid(*this);
            V.row(i) = vertex_attrs[i].pos;
        }

        F = Eigen::MatrixXi::Constant(tri_capacity(), 3, -1);
        for (auto& t : get_faces()) {
            auto i = t.fid(*this);
            auto vs = oriented_tri_vertices(t);
            for (int j = 0; j < 3; j++) {
                F(i, j) = vs[j].vid(*this);
            }
        }
    }

    void writeToVTU(const std::string& filename) const
    {
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        export_mesh(V, F);

        // get idx as Eigen::Matrix
        Eigen::MatrixXd vidx;
        vidx.resize(V.rows(), 1);
        for (unsigned i = 0; i < V.rows(); ++i) {
            vidx(i, 0) = vertex_attrs[i].idx;
        }

        Eigen::MatrixXd fidx;
        fidx.resize(F.rows(), 1);
        for (unsigned i = 0; i < F.rows(); ++i) {
            fidx(i, 0) = i;
        }

        paraviewo::VTUWriter writer;
        writer.add_field("vidx", vidx);
        writer.add_cell_field("fidx", fidx);
        writer.write_mesh(filename, V, F);
    }
};