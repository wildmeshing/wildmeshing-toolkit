#include <igl/edges.h>
#include <igl/oriented_facets.h>
#include <wmtk/TriMesh.h>
//#include <paraviewo/HDF5VTUWriter.hpp>
#include <paraviewo/ParaviewWriter.hpp>
#include <paraviewo/VTUWriter.hpp>
#include <wmtk/utils/Delaunay.hpp>
#include <wmtk/utils/io.hpp>


#include <catch2/catch_test_macros.hpp>

#include <sstream>

TEST_CASE("io", "[io][mshio]")
{
    using namespace wmtk;

    SECTION("Simple")
    {
        std::vector<Point3D> points{{{0, 0, 0}}, {{1, 0, 0}}, {{0, 1, 0}}, {{0, 0, 1}}};
        auto r = delaunay3D(points);
        const auto& vertices = std::get<0>(r);
        const auto& tets = std::get<1>(r);

        std::stringstream ss;

        MshData msh;

        // Edge visualization.
        msh.add_edge_vertices(vertices.size(), [&](size_t i) { return vertices[i]; });
        msh.add_edges(3, [&](size_t i) { return std::array<size_t, 2>{{0, i + 1}}; });
        msh.add_edge_vertex_attribute<1>("ev index", [&](size_t i) { return i; });
        msh.add_edge_attribute<1>("e index", [&](size_t i) { return i; });

        // Face visualization.
        msh.add_face_vertices(vertices.size(), [&](size_t i) { return vertices[i]; });
        msh.add_faces(4, [&](size_t i) -> std::array<size_t, 3> {
            if (i == 0) return {{1, 2, 3}};
            if (i == 1) return {{2, 0, 3}};
            if (i == 2) return {{0, 1, 3}};
            if (i == 3) return {{0, 2, 1}};
            throw std::runtime_error("Invalid index");
        });
        msh.add_face_vertex_attribute<1>("fv index", [&](size_t i) { return i; });
        msh.add_face_attribute<1>("f index", [&](size_t i) { return i; });

        // Tet visualization.
        msh.add_tet_vertices(vertices.size(), [&](size_t i) { return vertices[i]; });
        msh.add_tets(tets.size(), [&](size_t i) { return tets[i]; });
        msh.add_tet_vertex_attribute<1>("tv index", [&](size_t i) { return i; });
        msh.add_tet_attribute<1>("t index", [&](size_t i) { return i; });

        // Save and validate.
        msh.save(ss, true);
        MshData msh2;
        msh2.load(ss);

        REQUIRE(msh2.get_num_edge_vertices() == 4);
        REQUIRE(msh2.get_num_face_vertices() == 4);
        REQUIRE(msh2.get_num_tet_vertices() == 4);

        REQUIRE(msh2.get_num_edges() == 3);
        REQUIRE(msh2.get_num_faces() == 4);
        REQUIRE(msh2.get_num_tets() == 1);

        REQUIRE(msh2.get_edge_vertex_attribute_names().size() == 1);
        REQUIRE(msh2.get_face_vertex_attribute_names().size() == 1);
        REQUIRE(msh2.get_tet_vertex_attribute_names().size() == 1);

        REQUIRE(msh2.get_edge_attribute_names().size() == 1);
        REQUIRE(msh2.get_face_attribute_names().size() == 1);
        REQUIRE(msh2.get_tet_attribute_names().size() == 1);

        std::vector<Point3D> out_vertices;
        out_vertices.resize(msh.get_num_tet_vertices());
        msh.extract_tet_vertices([&](size_t i, double x, double y, double z) {
            out_vertices[i] = {{x, y, z}};
        });
        REQUIRE(out_vertices == vertices);

        std::vector<std::array<size_t, 4>> out_tets;
        out_tets.resize(msh.get_num_tets());
        msh.extract_tets([&](size_t i, size_t v0, size_t v1, size_t v2, size_t v3) {
            out_tets[i] = {{v0, v1, v2, v3}};
        });
        REQUIRE(out_tets == tets);

        std::vector<size_t> vertex_indices;
        vertex_indices.resize(msh.get_num_tet_vertices());
        msh.extract_tet_vertex_attribute(
            "tv index",
            [&](size_t i, const std::vector<double>& data) {
                REQUIRE(data.size() == 1);
                vertex_indices[i] = size_t(data[0]);
            });
        REQUIRE(vertex_indices == std::vector<size_t>({0, 1, 2, 3}));

        std::vector<size_t> tet_indices;
        tet_indices.resize(msh.get_num_tets());
        msh.extract_tet_attribute("t index", [&](size_t i, const std::vector<double>& data) {
            REQUIRE(data.size() == 1);
            tet_indices[i] = size_t(data[0]);
        });
        REQUIRE(tet_indices == std::vector<size_t>({0}));
    }
}

TEST_CASE("io-hang", "[io][mshio]")
{
    wmtk::MshData msh;
    REQUIRE_THROWS(msh.load(WMTK_DATA_DIR "nofile.msh"));
}


TEST_CASE("paraviewo-tri", "[io][paraviewo]")
{
    Eigen::MatrixXd vertices;
    vertices.resize(4, 2); // can be also 3D
    vertices << 0, 0, 1, 0, 0, 1, 1, 1;

    Eigen::MatrixXi faces;
    faces.resize(2, 3);
    faces << 0, 1, 2, 1, 3, 2;

    // paraviewo::HDF5VTUWriter writer;
    paraviewo::VTUWriter writer;

    SECTION("No attributes")
    {
        writer.write_mesh("triMesh_NoAttributes.vtu", vertices, faces);
    }

    SECTION("Add basic attributes")
    {
        // create some pseudo vertex attribute
        Eigen::MatrixXd vertex_idx;
        vertex_idx.resize(vertices.rows(), 1);
        for (unsigned i = 0; i < vertices.rows(); ++i) {
            vertex_idx(i, 0) = i;
        }

        // create some pseudo cell (in 2D that is a face) attribute
        Eigen::MatrixXd face_idx;
        face_idx.resize(faces.rows(), 1);
        for (unsigned i = 0; i < faces.rows(); ++i) {
            face_idx(i, 0) = i;
        }

        writer.add_field("vertex_idx", vertex_idx);
        writer.add_cell_field("face_idx", face_idx);
        writer.write_mesh("triMesh_BasicAttributes.vtu", vertices, faces);
    }

    // everything that is not a vertex or cell must be stored in its own file
    SECTION("Edge attributes")
    {
        Eigen::MatrixXi edges;
        igl::edges(faces, edges);

        // create some pseudo edge attribute
        Eigen::MatrixXd edge_idx;
        edge_idx.resize(edges.rows(), 1);
        for (unsigned i = 0; i < edges.rows(); ++i) {
            edge_idx(i, 0) = i;
        }

        // consider edges as cells
        writer.add_cell_field("edge_idx", edge_idx);
        writer.write_mesh("triMesh_EdgeAttributes.vtu", vertices, edges);
    }
}

TEST_CASE("paraviewo-tet", "[io][paraviewo]")
{
    Eigen::MatrixXd vertices;
    vertices.resize(5, 3);
    vertices << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, -1, 0, 0, 1;

    Eigen::MatrixXi tets;
    tets.resize(2, 4);
    tets << 0, 2, 1, 3, 0, 1, 2, 4;

    paraviewo::VTUWriter writer;

    SECTION("No attributes")
    {
        writer.write_mesh("tetMesh_NoAttributes.vtu", vertices, tets);
    }

    SECTION("Add basic attributes")
    {
        // create some pseudo vertex attribute
        Eigen::MatrixXd vertex_idx;
        vertex_idx.resize(vertices.rows(), 1);
        for (unsigned i = 0; i < vertices.rows(); ++i) {
            vertex_idx(i, 0) = i;
        }

        // create some pseudo cell (in 3D that is a tet) attribute
        Eigen::MatrixXd cell_idx;
        cell_idx.resize(tets.rows(), 1);
        for (unsigned i = 0; i < tets.rows(); ++i) {
            cell_idx(i, 0) = i;
        }

        writer.add_field("vertex_idx", vertex_idx);
        writer.add_cell_field("cell_idx", cell_idx);
        writer.write_mesh("tetMesh_BasicAttributes.vtu", vertices, tets);
    }

    // everything that is not a vertex or cell must be stored in its own file
    SECTION("Edge attributes")
    {
        Eigen::MatrixXi edges;
        igl::edges(tets, edges);

        // create some pseudo edge attribute
        Eigen::MatrixXd edge_idx;
        edge_idx.resize(edges.rows(), 1);
        for (unsigned i = 0; i < edges.rows(); ++i) {
            edge_idx(i, 0) = i;
        }

        // consider edges as cells
        writer.add_cell_field("edge_idx", edge_idx);
        writer.write_mesh("tetMesh_EdgeAttributes.vtu", vertices, edges);
    }

    SECTION("Face attributes")
    {
        Eigen::MatrixXi faces;
        igl::oriented_facets(tets, faces);


        // create some pseudo edge attribute
        Eigen::MatrixXd face_idx;
        face_idx.resize(faces.rows(), 1);
        for (unsigned i = 0; i < faces.rows(); ++i) {
            face_idx(i, 0) = i;
        }

        // consider faces as cells
        writer.add_cell_field("face_idx", face_idx);
        writer.write_mesh("tetMesh_FaceAttributes.vtu", vertices, faces);
    }
}