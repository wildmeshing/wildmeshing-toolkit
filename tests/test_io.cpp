#include <wmtk/utils/Delaunay.hpp>
#include <wmtk/utils/io.hpp>

#include <catch2/catch.hpp>

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

        auto pos2 = std::vector<Point3D>(msh2.get_num_tet_vertices());
        auto tets2 = std::vector<std::array<size_t, 4>>(msh2.get_num_tets());

        auto vert_setter = [&](size_t k, double x, double y, double z) { pos2[k] = {{x, y, z}}; };
        auto tet_setter = [&](size_t k, size_t v0, size_t v1, size_t v2, size_t v3) {
            tets2[k] = {{size_t(v0) - 1, size_t(v1) - 1, size_t(v2) - 1, size_t(v3) - 1}};
        };
        msh2.extract_tet_vertices(vert_setter);
        msh2.extract_tets(tet_setter);
        REQUIRE(std::equal(pos2.begin(), pos2.end(), points.begin(), points.end()));
        REQUIRE(std::equal(tets2.begin(), tets2.end(), tets.begin(), tets.end()));

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
    }
}
