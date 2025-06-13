#include <IncrementalTetWild.h>
#include <common.h>
#include <wmtk/TetMesh.h>

#include <igl/read_triangle_mesh.h>
#include <igl/write_triangle_mesh.h>
#include <catch2/catch_test_macros.hpp>
#include <wmtk/utils/io.hpp>
#include "spdlog/common.h"

using namespace wmtk;
using namespace tetwild;

TEST_CASE("link_count_vertex", "[link_count]")
{
    Parameters params;
    params.lr = 1 / 10.;
    params.init(Vector3d(0, 0, 0), Vector3d(1, 1, 1));

    wmtk::Envelope envelope;
    sample_envelope::SampleEnvelope sample_env;
    TetWild tetwild(params, envelope, sample_env);

    std::vector<VertexAttributes> vertices(7);
    vertices[0].m_posf = Vector3d(0, 0, 0);
    vertices[1].m_posf = Vector3d(1, 1, 1);
    vertices[2].m_posf = Vector3d(1, 1, 0);
    vertices[3].m_posf = Vector3d(1, 0, 0);
    vertices[4].m_posf = Vector3d(-1, -1, -1);
    vertices[5].m_posf = Vector3d(-1, -1, 0);
    vertices[6].m_posf = Vector3d(-1, 0, 0);
    std::vector<std::array<size_t, 4>> tets = {{{0, 1, 2, 3}}, {{0, 4, 5, 6}}};
    std::vector<TetAttributes> tet_attrs(1);
    for (auto& v : vertices) v.m_is_rounded = true;

    tetwild.init(vertices.size(), tets);
    tetwild.create_mesh_attributes(vertices, tet_attrs);

    for (int i = 0; i < tetwild.m_face_attribute.size(); i++) {
        tetwild.m_face_attribute[i].m_is_surface_fs = true;
    }

    // for (int i = 0; i < tetwild.m_face_attribute.size(); i++) {
    //     tetwild.m_face_attribute[i].m_is_surface_fs = false;
    // }

    // for (int i = 0; i < 4; i++) {
    //     tetwild.m_face_attribute[i].m_is_surface_fs = true;
    // }

    REQUIRE(tetwild.count_vertex_links(tetwild.tuple_from_vertex(0)) == 2);
}

TEST_CASE("link_count_vertex_2", "[link_count]")
{
    Parameters params;
    params.lr = 1 / 10.;
    params.init(Vector3d(0, 0, 0), Vector3d(1, 1, 1));

    wmtk::Envelope envelope;
    sample_envelope::SampleEnvelope sample_env;
    TetWild tetwild(params, envelope, sample_env);

    std::vector<VertexAttributes> vertices(10);
    vertices[0].m_posf = Vector3d(0, 0, 0);
    vertices[1].m_posf = Vector3d(1, 1, 1);
    vertices[2].m_posf = Vector3d(1, 1, 0);
    vertices[3].m_posf = Vector3d(1, 0, 0);
    vertices[4].m_posf = Vector3d(-1, -1, -1);
    vertices[5].m_posf = Vector3d(-1, -1, 0);
    vertices[6].m_posf = Vector3d(-1, 0, 0);
    vertices[7].m_posf = Vector3d(1, -1, 1);
    vertices[8].m_posf = Vector3d(1, -1, 0);
    vertices[9].m_posf = Vector3d(0, -1, 0);
    std::vector<std::array<size_t, 4>> tets = {{{0, 1, 2, 3}}, {{0, 4, 5, 6}}, {{0, 7, 8, 9}}};
    std::vector<TetAttributes> tet_attrs(1);
    for (auto& v : vertices) v.m_is_rounded = true;

    tetwild.init(vertices.size(), tets);
    tetwild.create_mesh_attributes(vertices, tet_attrs);

    for (int i = 0; i < tetwild.m_face_attribute.size(); i++) {
        tetwild.m_face_attribute[i].m_is_surface_fs = true;
    }

    // for (int i = 0; i < tetwild.m_face_attribute.size(); i++) {
    //     tetwild.m_face_attribute[i].m_is_surface_fs = false;
    // }

    // for (int i = 0; i < 4; i++) {
    //     tetwild.m_face_attribute[i].m_is_surface_fs = true;
    // }

    REQUIRE(tetwild.count_vertex_links(tetwild.tuple_from_vertex(0)) == 3);
}

TEST_CASE("link_count_edge", "[link_count]")
{
    Parameters params;
    params.lr = 1 / 10.;
    params.init(Vector3d(0, 0, 0), Vector3d(1, 1, 1));

    wmtk::Envelope envelope;
    sample_envelope::SampleEnvelope sample_env;
    TetWild tetwild(params, envelope, sample_env);

    std::vector<VertexAttributes> vertices(6);
    vertices[0].m_posf = Vector3d(0, 0, 0);
    vertices[1].m_posf = Vector3d(0, 0, 1);
    vertices[2].m_posf = Vector3d(1, 0, 0);
    vertices[3].m_posf = Vector3d(0, 1, 0);
    vertices[4].m_posf = Vector3d(-1, 0, 0);
    vertices[5].m_posf = Vector3d(0, -1, 0);
    std::vector<std::array<size_t, 4>> tets = {{{0, 1, 2, 3}}, {{0, 1, 4, 5}}};
    std::vector<TetAttributes> tet_attrs(1);
    for (auto& v : vertices) v.m_is_rounded = true;

    tetwild.init(vertices.size(), tets);
    tetwild.create_mesh_attributes(vertices, tet_attrs);

    for (int i = 0; i < tetwild.m_face_attribute.size(); i++) {
        tetwild.m_face_attribute[i].m_is_surface_fs = true;
    }

    // for (int i = 0; i < 4; i++) {
    //     tetwild.m_face_attribute[i].m_is_surface_fs = true;
    // }

    REQUIRE(tetwild.count_edge_links(tetwild.tuple_from_edge(0, 0)) == 4);
}


TEST_CASE("link_count_edge_2", "[link_count]")
{
    Parameters params;
    params.lr = 1 / 10.;
    params.init(Vector3d(0, 0, 0), Vector3d(1, 1, 1));

    wmtk::Envelope envelope;
    sample_envelope::SampleEnvelope sample_env;
    TetWild tetwild(params, envelope, sample_env);

    std::vector<VertexAttributes> vertices(8);
    vertices[0].m_posf = Vector3d(0, 0, 0);
    vertices[1].m_posf = Vector3d(0, 0, 1);
    vertices[2].m_posf = Vector3d(1, 0, 0);
    vertices[3].m_posf = Vector3d(0, 1, 0);
    vertices[4].m_posf = Vector3d(-1, 0, 0);
    vertices[5].m_posf = Vector3d(0, -1, 0);
    vertices[6].m_posf = Vector3d(1, -1, 0);
    vertices[7].m_posf = Vector3d(0.5, -1, 0);
    std::vector<std::array<size_t, 4>> tets = {{{0, 1, 2, 3}}, {{0, 1, 4, 5}}, {{0, 1, 6, 7}}};
    std::vector<TetAttributes> tet_attrs(1);
    for (auto& v : vertices) v.m_is_rounded = true;

    tetwild.init(vertices.size(), tets);
    tetwild.create_mesh_attributes(vertices, tet_attrs);

    for (int i = 0; i < tetwild.m_face_attribute.size(); i++) {
        tetwild.m_face_attribute[i].m_is_surface_fs = true;
    }

    // for (int i = 0; i < 4; i++) {
    //     tetwild.m_face_attribute[i].m_is_surface_fs = true;
    // }

    REQUIRE(tetwild.count_edge_links(tetwild.tuple_from_edge(0, 0)) == 6);
}

TEST_CASE("link_count_edge_3", "[link_count]")
{
    Parameters params;
    params.lr = 1 / 10.;
    params.init(Vector3d(0, 0, 0), Vector3d(1, 1, 1));

    wmtk::Envelope envelope;
    sample_envelope::SampleEnvelope sample_env;
    TetWild tetwild(params, envelope, sample_env);

    std::vector<VertexAttributes> vertices(8);
    vertices[0].m_posf = Vector3d(0, 0, 0);
    vertices[1].m_posf = Vector3d(0, 0, 1);
    vertices[2].m_posf = Vector3d(1, 0, 0);
    vertices[3].m_posf = Vector3d(0, 1, 0);
    vertices[4].m_posf = Vector3d(-1, 0, 0);
    vertices[5].m_posf = Vector3d(0, -1, 0);
    std::vector<std::array<size_t, 4>> tets = {{{0, 1, 2, 3}}, {{0, 1, 4, 5}}, {{0, 1, 2, 5}}};
    std::vector<TetAttributes> tet_attrs(1);
    for (auto& v : vertices) v.m_is_rounded = true;

    tetwild.init(vertices.size(), tets);
    tetwild.create_mesh_attributes(vertices, tet_attrs);

    for (int i = 0; i < tetwild.m_face_attribute.size(); i++) {
        tetwild.m_face_attribute[i].m_is_surface_fs = true;
    }

    // for (int i = 0; i < 4; i++) {
    //     tetwild.m_face_attribute[i].m_is_surface_fs = true;
    // }

    REQUIRE(tetwild.count_edge_links(tetwild.tuple_from_edge(0, 0)) == 4);
}
