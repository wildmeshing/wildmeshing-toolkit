#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include <wmtk/operations/OperationFactory.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplit.hpp>

#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"

#include <catch2/catch_test_macros.hpp>

#include <igl/read_triangle_mesh.h>

using namespace wmtk;
using namespace wmtk::tests;


constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;

TEST_CASE("hdf5_2d", "[io]")
{
    RowVectors3l tris;
    tris.resize(1, 3);
    tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};

    TriMesh mesh;
    mesh.initialize(tris);

    HDF5Writer writer("test.hdf5");
    mesh.serialize(writer);
}

TEST_CASE("hdf5_2d_read", "[io]")
{
    RowVectors3l tris;
    tris.resize(1, 3);
    tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};

    TriMesh mesh, mesh1;
    mesh.initialize(tris);

    HDF5Writer writer("test.hdf5");
    mesh.serialize(writer);

    MeshReader reader("test.hdf5");
    reader.read(mesh1);

    CHECK(mesh1 == mesh);
}

TEST_CASE("paraview_2d", "[io]")
{
    Eigen::MatrixXd V;
    Eigen::Matrix<long, -1, -1> F;

    igl::read_triangle_mesh(WMTK_DATA_DIR "/fan.obj", V, F);

    TriMesh mesh;
    mesh.initialize(F);

    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, mesh);

    ParaviewWriter writer("paraview", "vertices", mesh, true, true, true, false);
    mesh.serialize(writer);
}

TEST_CASE("hdf5_3d", "[io]")
{
    Eigen::Matrix<long, 2, 4> T;
    T << 0, 1, 2, 3, 4, 5, 6, 7;
    TetMesh mesh;
    mesh.initialize(T);

    HDF5Writer writer("test.hdf5");
    mesh.serialize(writer);

    MeshReader reader("test.hdf5");
}

TEST_CASE("paraview_3d", "[io]")
{
    Eigen::Matrix<long, 2, 4> T;
    T << 0, 1, 2, 3, 4, 5, 6, 7;
    TetMesh mesh;
    mesh.initialize(T);
    Eigen::MatrixXd V(8, 3);
    V.setRandom();
    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, mesh);

    ParaviewWriter writer("paraview", "vertices", mesh, true, true, true, true);
    mesh.serialize(writer);
}

TEST_CASE("attribute_after_split", "[io]")
{
    DEBUG_TriMesh m = single_triangle_with_position();
    wmtk::MeshAttributeHandle<long> attribute_handle =
        m.register_attribute<long>(std::string("test_attribute"), PE, 1);
    wmtk::MeshAttributeHandle<double> pos_handle =
        m.get_attribute_handle<double>(std::string("position"), PV);

    {
        Accessor<long> acc_attribute = m.create_accessor<long>(attribute_handle);
        Accessor<double> acc_pos = m.create_accessor<double>(pos_handle);

        const Tuple edge = m.edge_tuple_between_v1_v2(0, 1, 0);

        Eigen::Vector3d p_mid;
        {
            const Eigen::Vector3d p0 = acc_pos.vector_attribute(edge);
            const Eigen::Vector3d p1 = acc_pos.vector_attribute(m.switch_vertex(edge));
            p_mid = 0.5 * (p0 + p1);
        }

        {
            // set edge(0,1)'s tag as 1
            acc_attribute.scalar_attribute(edge) = 1;

            // all edges hold 0 besides "edge"
            for (const Tuple& t : m.get_all(PE)) {
                if (m.simplices_are_equal(Simplex::edge(edge), Simplex::edge(t))) {
                    CHECK(acc_attribute.scalar_attribute(t) == 1);
                } else {
                    CHECK(acc_attribute.scalar_attribute(t) == 0);
                }
            }

            wmtk::operations::OperationSettings<operations::tri_mesh::EdgeSplit> op_settings;
            op_settings.split_boundary_edges = true;
            op_settings.initialize_invariants(m);

            operations::tri_mesh::EdgeSplit op(m, edge, op_settings);
            REQUIRE(op());

            // set new vertex position
            acc_pos.vector_attribute(op.return_tuple()) = p_mid;
        }

        // since the default value is 0, there should be no other value in this triangle
        for (const Tuple& t : m.get_all(PE)) {
            CHECK(acc_attribute.scalar_attribute(t) == 0);
        }
    } // end of scope for the accessors

    {
        Accessor<long> acc_attribute = m.create_accessor<long>(attribute_handle);
        for (const Tuple& t : m.get_all(PE)) {
            CHECK(acc_attribute.scalar_attribute(t) == 0);
        }
    }

    // attribute_after_split_edges.hdf contains a 1 in the "test_attribute"
    ParaviewWriter writer("attribute_after_split", "position", m, true, true, true, false);
    m.serialize(writer);
}

// TEST_CASE("io", "[io][mshio]")
// {
//     using namespace wmtk;

//     SECTION("Simple")
//     {
//         std::vector<Point3D> points{{{0, 0, 0}}, {{1, 0, 0}}, {{0, 1, 0}}, {{0, 0, 1}}};
//         auto r = delaunay3D(points);
//         const auto& vertices = std::get<0>(r);
//         const auto& tets = std::get<1>(r);

//         std::stringstream ss;

//         MshData msh;

//         // Edge visualization.
//         msh.add_edge_vertices(vertices.size(), [&](size_t i) { return vertices[i]; });
//         msh.add_edges(3, [&](size_t i) { return std::array<size_t, 2>{{0, i + 1}}; });
//         msh.add_edge_vertex_attribute<1>("ev index", [&](size_t i) { return i; });
//         msh.add_edge_attribute<1>("e index", [&](size_t i) { return i; });

//         // Face visualization.
//         msh.add_face_vertices(vertices.size(), [&](size_t i) { return vertices[i]; });
//         msh.add_faces(4, [&](size_t i) -> std::array<size_t, 3> {
//             if (i == 0) return {{1, 2, 3}};
//             if (i == 1) return {{2, 0, 3}};
//             if (i == 2) return {{0, 1, 3}};
//             if (i == 3) return {{0, 2, 1}};
//             throw std::runtime_error("Invalid index");
//         });
//         msh.add_face_vertex_attribute<1>("fv index", [&](size_t i) { return i; });
//         msh.add_face_attribute<1>("f index", [&](size_t i) { return i; });

//         // Tet visualization.
//         msh.add_tet_vertices(vertices.size(), [&](size_t i) { return vertices[i]; });
//         msh.add_tets(tets.size(), [&](size_t i) { return tets[i]; });
//         msh.add_tet_vertex_attribute<1>("tv index", [&](size_t i) { return i; });
//         msh.add_tet_attribute<1>("t index", [&](size_t i) { return i; });

//         // Save and validate.
//         msh.save(ss, true);
//         MshData msh2;
//         msh2.load(ss);

//         REQUIRE(msh2.get_num_edge_vertices() == 4);
//         REQUIRE(msh2.get_num_face_vertices() == 4);
//         REQUIRE(msh2.get_num_tet_vertices() == 4);

//         REQUIRE(msh2.get_num_edges() == 3);
//         REQUIRE(msh2.get_num_faces() == 4);
//         REQUIRE(msh2.get_num_tets() == 1);

//         REQUIRE(msh2.get_edge_vertex_attribute_names().size() == 1);
//         REQUIRE(msh2.get_face_vertex_attribute_names().size() == 1);
//         REQUIRE(msh2.get_tet_vertex_attribute_names().size() == 1);

//         REQUIRE(msh2.get_edge_attribute_names().size() == 1);
//         REQUIRE(msh2.get_face_attribute_names().size() == 1);
//         REQUIRE(msh2.get_tet_attribute_names().size() == 1);

//         std::vector<Point3D> out_vertices;
//         out_vertices.resize(msh.get_num_tet_vertices());
//         msh.extract_tet_vertices([&](size_t i, double x, double y, double z) {
//             out_vertices[i] = {{x, y, z}};
//         });
//         REQUIRE(out_vertices == vertices);

//         std::vector<std::array<size_t, 4>> out_tets;
//         out_tets.resize(msh.get_num_tets());
//         msh.extract_tets([&](size_t i, size_t v0, size_t v1, size_t v2, size_t v3) {
//             out_tets[i] = {{v0, v1, v2, v3}};
//         });
//         REQUIRE(out_tets == tets);

//         std::vector<size_t> vertex_indices;
//         vertex_indices.resize(msh.get_num_tet_vertices());
//         msh.extract_tet_vertex_attribute(
//             "tv index",
//             [&](size_t i, const std::vector<double>& data) {
//                 REQUIRE(data.size() == 1);
//                 vertex_indices[i] = size_t(data[0]);
//             });
//         REQUIRE(vertex_indices == std::vector<size_t>({0, 1, 2, 3}));

//         std::vector<size_t> tet_indices;
//         tet_indices.resize(msh.get_num_tets());
//         msh.extract_tet_attribute("t index", [&](size_t i, const std::vector<double>& data) {
//             REQUIRE(data.size() == 1);
//             tet_indices[i] = size_t(data[0]);
//         });
//         REQUIRE(tet_indices == std::vector<size_t>({0}));
//     }
// }

// TEST_CASE("io-hang", "[io][mshio]")
// {
//     wmtk::MshData msh;
//     REQUIRE_THROWS(msh.load(WMTK_DATA_DIR "nofile.msh"));
// }


// TEST_CASE("paraviewo-tri", "[io][paraviewo]")
// {
//     Eigen::MatrixXd vertices;
//     vertices.resize(4, 2); // can be also 3D
//     vertices << 0, 0, 1, 0, 0, 1, 1, 1;

//     Eigen::MatrixXi faces;
//     faces.resize(2, 3);
//     faces << 0, 1, 2, 1, 3, 2;

//     paraviewo::HDF5VTUWriter writer;

//     SECTION("No attributes")
//     {
//         writer.write_mesh("triMesh_NoAttributes.hdf", vertices, faces);
//     }

//     SECTION("Add basic attributes")
//     {
//         // create some pseudo vertex attribute
//         Eigen::MatrixXd vertex_idx;
//         vertex_idx.resize(vertices.rows(), 1);
//         for (unsigned i = 0; i < vertices.rows(); ++i) {
//             vertex_idx(i, 0) = i;
//         }

//         // create some pseudo cell (in 2D that is a face) attribute
//         Eigen::MatrixXd face_idx;
//         face_idx.resize(faces.rows(), 1);
//         for (unsigned i = 0; i < faces.rows(); ++i) {
//             face_idx(i, 0) = i;
//         }

//         writer.add_field("vertex_idx", vertex_idx);
//         writer.add_cell_field("face_idx", face_idx);
//         writer.write_mesh("triMesh_BasicAttributes.hdf", vertices, faces);
//     }

//     // everything that is not a vertex or cell must be stored in its own file
//     SECTION("Edge attributes")
//     {
//         Eigen::MatrixXi edges;
//         igl::edges(faces, edges);

//         // create some pseudo edge attribute
//         Eigen::MatrixXd edge_idx;
//         edge_idx.resize(edges.rows(), 1);
//         for (unsigned i = 0; i < edges.rows(); ++i) {
//             edge_idx(i, 0) = i;
//         }

//         // consider edges as cells
//         writer.add_cell_field("edge_idx", edge_idx);
//         writer.write_mesh("triMesh_EdgeAttributes.hdf", vertices, edges);
//     }
// }

// TEST_CASE("paraviewo-tet", "[io][paraviewo]")
// {
//     Eigen::MatrixXd vertices;
//     vertices.resize(5, 3);
//     vertices << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, -1, 0, 0, 1;

//     Eigen::MatrixXi tets;
//     tets.resize(2, 4);
//     tets << 0, 2, 1, 3, 0, 1, 2, 4;

//     paraviewo::HDF5VTUWriter writer;

//     SECTION("No attributes")
//     {
//         writer.write_mesh("tetMesh_NoAttributes.hdf", vertices, tets);
//     }

//     SECTION("Add basic attributes")
//     {
//         // create some pseudo vertex attribute
//         Eigen::MatrixXd vertex_idx;
//         vertex_idx.resize(vertices.rows(), 1);
//         for (unsigned i = 0; i < vertices.rows(); ++i) {
//             vertex_idx(i, 0) = i;
//         }

//         // create some pseudo cell (in 3D that is a tet) attribute
//         Eigen::MatrixXd cell_idx;
//         cell_idx.resize(tets.rows(), 1);
//         for (unsigned i = 0; i < tets.rows(); ++i) {
//             cell_idx(i, 0) = i;
//         }

//         writer.add_field("vertex_idx", vertex_idx);
//         writer.add_cell_field("cell_idx", cell_idx);
//         writer.write_mesh("tetMesh_BasicAttributes.hdf", vertices, tets);
//     }

//     // everything that is not a vertex or cell must be stored in its own file
//     SECTION("Edge attributes")
//     {
//         Eigen::MatrixXi edges;
//         igl::edges(tets, edges);

//         // create some pseudo edge attribute
//         Eigen::MatrixXd edge_idx;
//         edge_idx.resize(edges.rows(), 1);
//         for (unsigned i = 0; i < edges.rows(); ++i) {
//             edge_idx(i, 0) = i;
//         }

//         // consider edges as cells
//         writer.add_cell_field("edge_idx", edge_idx);
//         writer.write_mesh("tetMesh_EdgeAttributes.hdf", vertices, edges);
//     }

//     SECTION("Face attributes")
//     {
//         Eigen::MatrixXi faces;
//         igl::oriented_facets(tets, faces);


//         // create some pseudo edge attribute
//         Eigen::MatrixXd face_idx;
//         face_idx.resize(faces.rows(), 1);
//         for (unsigned i = 0; i < faces.rows(); ++i) {
//             face_idx(i, 0) = i;
//         }

//         // consider faces as cells
//         writer.add_cell_field("face_idx", face_idx);
//         writer.write_mesh("tetMesh_FaceAttributes.hdf", vertices, faces);
//     }
// }

// TEST_CASE("h5pp", "[io][h5pp]")
// {
//     // Initialize a file
//     h5pp::File file("exampledir/example-03a-attributes-readwrite.h5", h5pp::FileAccess::REPLACE);

//     // Write an integer to file

//     file.writeDataset(42, "intGroup/myInt");

//     // We can now add attributes to the dataset
//     file.writeAttribute(
//         "this is some info about my int",
//         "intGroup/myInt",
//         "myInt_stringAttribute");
//     file.writeAttribute(3.14, "intGroup/myInt", "myInt_doubleAttribute");

//     // List all attributes associated with our dataset. The following will be printed:
//     //      {"myInt_stringAttribute", "myInt_doubleAttribute"}
//     h5pp::print("{}\n", file.getAttributeNames("intGroup/myInt"));

//     // Read the attribute data back
//     auto stringAttribute =
//         file.readAttribute<std::string>("intGroup/myInt", "myInt_stringAttribute");
//     auto doubleAttribute = file.readAttribute<double>("intGroup/myInt", "myInt_doubleAttribute");

//     // Print the data
//     h5pp::print("stringAttribute read: {}\n", stringAttribute);
//     h5pp::print("doubleAttribute read: {}\n", doubleAttribute);
// }
