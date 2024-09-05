#include <catch2/catch_test_macros.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/VolumeRemesherTriangleInsertion.cpp>
#include <wmtk/utils/VolumeRemesherTriangleInsertion.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "tools/DEBUG_TetMesh.hpp"
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"

const std::filesystem::path data_dir = WMTK_DATA_DIR;

using namespace wmtk::utils;
using namespace wmtk::tests;
using namespace wmtk;

TEST_CASE("test_background_mesh", "[triangleinsertion][.]")
{
    RowVectors4l background_TV;
    RowVectors3d background_V;
    generate_background_mesh(
        Vector3d(0, 0, 0),
        Vector3d(1, 1, 1),
        {{5, 5, 5}},
        background_TV,
        background_V);

    TetMesh m;
    m.initialize(background_TV);
    mesh_utils::set_matrix_attribute(background_V, "vertices", PrimitiveType::Vertex, m);

    ParaviewWriter writer("test_background_mesh", "vertices", m, true, true, true, true);
    m.serialize(writer);
}

TEST_CASE("test_insert_surface_mesh", "[triangleinsertion][.]")
{
    // std::shared_ptr<Mesh> m = read_mesh(WMTK_DATA_DIR "/sphere.msh");
    // tests::DEBUG_TriMesh& surface_mesh = static_cast<tests::DEBUG_TriMesh&>(*m);

    DEBUG_TriMesh surface_mesh = hex_plus_two_with_position();
    RowVectors3d V(surface_mesh.capacity(PrimitiveType::Vertex), 3);
    RowVectors3l F(surface_mesh.capacity(PrimitiveType::Triangle), 3);

    auto pos_handle = surface_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto pos_accessor = surface_mesh.create_accessor(pos_handle.as<double>());

    auto vs = surface_mesh.get_all(PrimitiveType::Vertex);
    for (int64_t i = 0; i < vs.size(); ++i) {
        V.row(i) = pos_accessor.vector_attribute(vs[i]);
    }

    for (int64_t i = 0; i < surface_mesh.capacity(PrimitiveType::Triangle); ++i) {
        F.row(i) = surface_mesh.fv_from_fid(i);
    }

    auto [tetmesh, tet_local_faces] = generate_raw_tetmesh_from_input_surface(V, F, 0.1);

    ParaviewWriter writer("test_insertion", "vertices", *tetmesh, true, true, true, true);
    tetmesh->serialize(writer);
}

TEST_CASE("test_insert_and_register_surface_mesh", "[triangleinsertion][.]")
{
    std::shared_ptr<Mesh> m = read_mesh(WMTK_DATA_DIR "/sphere.msh");
    tests::DEBUG_TriMesh& surface_mesh = static_cast<tests::DEBUG_TriMesh&>(*m);

    // DEBUG_TriMesh surface_mesh = hex_plus_two_with_position();
    RowVectors3d V(surface_mesh.capacity(PrimitiveType::Vertex), 3);
    RowVectors3l F(surface_mesh.capacity(PrimitiveType::Triangle), 3);

    auto pos_handle = surface_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto pos_accessor = surface_mesh.create_accessor(pos_handle.as<double>());

    auto vs = surface_mesh.get_all(PrimitiveType::Vertex);
    for (int64_t i = 0; i < vs.size(); ++i) {
        V.row(i) = pos_accessor.vector_attribute(vs[i]);
    }

    for (int64_t i = 0; i < surface_mesh.capacity(PrimitiveType::Triangle); ++i) {
        F.row(i) = surface_mesh.fv_from_fid(i);
    }

    auto [tetmesh, facemesh] = generate_raw_tetmesh_with_surface_from_input(V, F, 0.1);

    ParaviewWriter writer("test_mm_insertion", "vertices", *tetmesh, true, true, true, true);
    tetmesh->serialize(writer);

    auto parent_pos_handle =
        tetmesh->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto parent_pos_accessor = tetmesh->create_accessor(parent_pos_handle.as<double>());

    auto child_pos_handle =
        facemesh->register_attribute<double>("vertices", PrimitiveType::Vertex, 3);
    auto child_pos_accessor = facemesh->create_accessor(child_pos_handle.as<double>());

    for (const auto& v : facemesh->get_all(PrimitiveType::Vertex)) {
        child_pos_accessor.vector_attribute(v) = parent_pos_accessor.vector_attribute(
            facemesh->map_to_parent_tuple(simplex::Simplex::vertex(*facemesh, v)));
    }

    ParaviewWriter
        writer2("test_mm_insertion_surface", "vertices", *facemesh, true, true, true, false);
    facemesh->serialize(writer2);
}
