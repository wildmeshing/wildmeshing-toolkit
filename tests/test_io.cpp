#include <igl/edges.h>
#include <igl/oriented_facets.h>
#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>
#include <wmtk/io/TetVTUWriter.hpp>
#include <wmtk/io/TriVTUWriter.hpp>
#include <wmtk/utils/Delaunay.hpp>
#include <wmtk/utils/examples/TetMesh_examples.hpp>
#include <wmtk/utils/examples/TriMesh_examples.hpp>
#include <wmtk/utils/io.hpp>


#include <catch2/catch_test_macros.hpp>

#include <sstream>

using namespace wmtk;
using namespace wmtk::utils::examples;

TEST_CASE("io", "[io][mshio]")
{
    using namespace wmtk;

    SECTION("Simple")
    {
        std::vector<delaunay::Point3D> points{{{0, 0, 0}}, {{1, 0, 0}}, {{0, 1, 0}}, {{0, 0, 1}}};
        auto r = delaunay::delaunay3D(points);
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

        std::vector<delaunay::Point3D> out_vertices;
        out_vertices.resize(msh.get_num_tet_vertices());
        msh.extract_tet_vertices(
            [&](size_t i, double x, double y, double z) { out_vertices[i] = {{x, y, z}}; });
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
    tri::TriMeshVF VF = tri::edge_region();

    TriMesh m;
    m.init(VF.F);

    io::TriVTUWriter writer(m);
    writer.add_vertex_positions([&m, &VF](int i) { return VF.V.row(i); });
    writer.add_vertex_attribute("position", [&m, &VF](int i) { return VF.V.row(i); });
    writer.add_vertex_attribute("vid", [&m](int i) { return VectorXd::Constant(1, i); });
    writer.add_triangle_attribute("fid", [&m](int i) { return VectorXd::Constant(1, i); });
    writer.add_edge_attribute("eid", [&m](int i) { return VectorXd::Constant(1, i); });
    writer.write_triangles("tri_paraviewo_f.vtu");
    writer.write_edges("tri_paraviewo_e.vtu");
}

TEST_CASE("paraviewo-tet", "[io][paraviewo]")
{
    tet::TetMeshVT VT = tet::six_cycle_tets();

    TetMesh m;
    m.init(VT.T);

    io::TetVTUWriter writer(m);
    writer.add_vertex_positions([&m, &VT](int i) { return VT.V.row(i); });

    writer.add_vertex_attribute("position", [&m, &VT](int i) { return VT.V.row(i); });
    writer.add_vertex_attribute("vid", [&m](int i) { return VectorXd::Constant(1, i); });

    writer.add_tet_attribute("tid", [&m](int i) { return VectorXd::Constant(1, i); });
    writer.add_triangle_attribute("fid", [&m](int i) { return VectorXd::Constant(1, i); });
    writer.add_edge_attribute("eid", [&m](int i) { return VectorXd::Constant(1, i); });

    writer.write_tets("tet_paraviewo_t.vtu");
    writer.write_triangles("tet_paraviewo_f.vtu");
    writer.write_edges("tet_paraviewo_e.vtu");
}

TEST_CASE("io-pysical-groups", "[io][mshio][.]")
{
    tri::TriMeshVF VF = tri::edge_region(3);
    const auto& V = VF.V;
    const auto& F = VF.F;

    wmtk::MshData msh;
    msh.add_face_vertices(V.rows(), [&](size_t k) -> Vector3d { return V.row(k); });

    const size_t n_tet_vertices = V.rows();

    msh.add_faces(F.rows(), [&](size_t k) { return F.row(k); });

    msh.add_physical_group("testgroup"); // does not work with binary for now

    msh.save("test-io-pysical-groups.msh", true);
}