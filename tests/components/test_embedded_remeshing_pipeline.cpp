#include <stdio.h>
#include <stdlib.h>
#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <nlohmann/json.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplit.hpp>
#include <wmtk/operations/tri_mesh/VertexPushOffset.hpp>
#include <wmtk/operations/utils/HelperFunctions.hpp>
#include <wmtk/simplex/link.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk_components/embedded_remeshing/internal/ModelLoader.hpp>
#include <wmtk_components/marching/internal/Marching.hpp>
#include <wmtk_components/mesh_info/mesh_info.hpp>
#include <wmtk_components/regular_space/internal/RegularSpace.hpp>
#include <wmtk_components/regular_space/regular_space.hpp>
#include "wmtk/../../tests/tools/DEBUG_TetMesh.hpp"
#include "wmtk/../../tests/tools/DEBUG_TriMesh.hpp"
#include "wmtk/../../tests/tools/TetMesh_examples.hpp"
#include "wmtk/../../tests/tools/TriMesh_examples.hpp"

using json = nlohmann::json;
using namespace wmtk;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("matrix_load", "[pipeline][.]")
{
    SECTION("trimesh_load")
    {
        std::vector<std::vector<long>> labels;
        for (long j = 0; j < 22; ++j) {
            std::vector<long> line;
            line.reserve(20);
            for (long i = 0; i < 20; ++i) {
                if ((i - 10) * (i - 10) + (j - 11) * (j - 11) < 36) {
                    line.push_back(1);
                } else {
                    line.push_back(0);
                }
            }
            labels.push_back(line);
        }
        TriMesh mesh;
        wmtk::components::internal::load_matrix_in_trimesh(mesh, labels);
        if (true) {
            ParaviewWriter
                writer(data_dir / "trimesh_matrix_load", "position", mesh, true, true, true, false);
            mesh.serialize(writer);
        }
    }
    SECTION("tetmesh_load")
    {
        std::vector<std::vector<std::vector<long>>> labels;
        for (long k = 0; k < 24; ++k) {
            std::vector<std::vector<long>> layer;
            for (long j = 0; j < 22; ++j) {
                std::vector<long> line;
                line.reserve(20);
                for (long i = 0; i < 20; ++i) {
                    if ((i - 10) * (i - 10) + (j - 11) * (j - 11) + (k - 12) * (k - 12) < 36) {
                        line.push_back(1);
                    } else {
                        line.push_back(0);
                    }
                }
                layer.push_back(line);
            }
            labels.push_back(layer);
        }
        TetMesh mesh;
        wmtk::components::internal::load_matrix_in_tetmesh(mesh, labels);
        if (true) {
            ParaviewWriter
                writer(data_dir / "tetmesh_matrix_load", "position", mesh, true, true, true, true);
            mesh.serialize(writer);
        }
    }
}

TEST_CASE("embedded_remeshing_relocation_helper_function_invert", "[pipeline][.]")
{
    SECTION("invert_2D-should_fail")
    {
        //   3 ----------- 4
        //   |  \        / |
        //   |   \  f3  /  |
        //   |    \    /   |
        //   |     \  /    |
        //   |      0      |
        //   |     / \     |
        //   | f1 /   \ f2 |
        //   |   / f0  \   |
        //   |  /       \  |
        //   1  ---------  2
        //
        tests::DEBUG_TriMesh mesh = wmtk::tests::two_neighbors_plus_one();
        Eigen::MatrixXd V(5, 3);
        V.row(0) << 0, 0, 0;
        V.row(1) << -1, -1, 0;
        V.row(2) << 1, -1, 0;
        V.row(3) << -1, 1, 0;
        V.row(4) << 1, 1, 0;
        MeshAttributeHandle<double> pos_handle =
            mesh_utils::set_matrix_attribute(V, "position", PrimitiveType::Vertex, mesh);

        CHECK(!wmtk::operations::utils::is_invert(
            mesh,
            pos_handle,
            mesh.get_all(PrimitiveType::Vertex)[0],
            PrimitiveType::Face));
    }
    SECTION("invert_2D-should_pass")
    {
        //   3 ----------- 4
        //   |  \        / |
        //   |   \  f3  /  |
        //   |    \    /   |
        //   |     \  /    |
        //   |      0      |
        //   |     / \     |
        //   | f1 /   \ f2 |
        //   |   / f0  \   |
        //   |  /       \  |
        //   1  ---------  2
        //
        tests::DEBUG_TriMesh mesh = wmtk::tests::two_neighbors_plus_one();
        Eigen::MatrixXd V(5, 3);
        V.row(0) << 5, 0, 0;
        V.row(1) << -1, -1, 0;
        V.row(2) << 1, -1, 0;
        V.row(3) << -1, 1, 0;
        V.row(4) << 1, 1, 0;
        MeshAttributeHandle<double> pos_handle =
            mesh_utils::set_matrix_attribute(V, "position", PrimitiveType::Vertex, mesh);

        CHECK(wmtk::operations::utils::is_invert(
            mesh,
            pos_handle,
            mesh.get_all(PrimitiveType::Vertex)[0],
            PrimitiveType::Face));
    }
}

TEST_CASE("embedded_remeshing_relocation_helper_function_push", "[pipeline][.]")
{
    SECTION("helper_function_push")
    {
        //    0---1---2
        //   /0\1/2\3/4\ .
        //  3---4---5---6
        //   \5/6\7/  .
        //    7---8
        tests::DEBUG_TriMesh m = wmtk::tests::hex_plus_two_with_position();
        MeshAttributeHandle<long> edge_tag_handle =
            m.register_attribute<long>("edge_tag", PrimitiveType::Edge, 1);
        MeshAttributeHandle<long> vertex_tag_handle =
            m.register_attribute<long>("vertex_tag", PrimitiveType::Vertex, 1);
        MeshAttributeHandle<long> todo_tag_handle =
            m.register_attribute<long>("todo_tag", PrimitiveType::Vertex, 1, false, 1);
        Accessor<long> acc_edge_tag = m.create_accessor(edge_tag_handle);
        Accessor<long> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
        acc_vertex_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[4]) = 2;
        acc_vertex_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[5]) = 1;
        acc_vertex_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[6]) = 1;
        acc_vertex_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[7]) = 1;
        acc_vertex_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[8]) = 1;
        acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(7, 8, 6)) = 1;
        acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(5, 8, 7)) = 1;
        acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(5, 6, 4)) = 1;
        // wmtk::simplex::SimplexCollection sc =
        //     simplex::link(m, Simplex(PrimitiveType::Vertex,
        //     m.get_all(PrimitiveType::Vertex)[4]));
        // int size = 0;
        // for (const Simplex& s : sc.simplex_vector(PrimitiveType::Edge)) {
        //     CHECK(acc_edge_tag.scalar_attribute(s.tuple()) == 1);
        //     ++size;
        // }
        // CHECK(size == 6);

        operations::OperationSettings<operations::tri_mesh::VertexPushOffset> settings;
        settings.edge_tag_handle = edge_tag_handle;
        settings.embedding_tag_value = 0;
        settings.input_tag_value = 1;
        settings.offset_len = 5;
        settings.offset_tag_value = 2;
        settings.position = m.get_attribute_handle<double>("position", PrimitiveType::Vertex);
        settings.vertex_tag_handle = vertex_tag_handle;
        settings.todo_tag_handle = todo_tag_handle;
        settings.initialize_invariants(m);
        operations::tri_mesh::VertexPushOffset op(m, m.get_all(PrimitiveType::Vertex)[4], settings);
        op();

        if (true) {
            ParaviewWriter writer(data_dir / "push_result", "position", m, true, true, true, false);
            m.serialize(writer);
        }
    }
}

TEST_CASE("embedded_remeshing_2D_pipeline", "[pipeline][2D][.]")
{
    std::vector<std::vector<long>> labels;
    for (long j = 0; j < 22; ++j) {
        std::vector<long> line;
        line.reserve(20);
        for (long i = 0; i < 20; ++i) {
            if ((i - 10) * (i - 10) + (j - 11) * (j - 11) < 36) {
                line.push_back(1);
            } else {
                line.push_back(0);
            }
        }
        labels.push_back(line);
    }
    TriMesh mesh;
    wmtk::components::internal::load_matrix_in_trimesh(mesh, labels);

    MeshAttributeHandle<double> pos_handle =
        mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    MeshAttributeHandle<long> vertex_tag_hanlde =
        mesh.get_attribute_handle<long>("vertex_tag", PrimitiveType::Vertex);
    MeshAttributeHandle<long> edge_tag_handle =
        mesh.get_attribute_handle<long>("edge_tag", PrimitiveType::Edge);

    components::internal::RegularSpace rs(pos_handle, vertex_tag_hanlde, edge_tag_handle, 1, 0, 2);
    rs.process_edge_simplicity_in_2d(mesh);

    MeshAttributeHandle<long> todo_handle =
        mesh.register_attribute<long>("todo_tag", PrimitiveType::Edge, 1, false, 1);

    components::internal::Marching
        mc(pos_handle, vertex_tag_hanlde, edge_tag_handle, todo_handle, 1, 0, 2);
    mc.process(mesh);

    Accessor<long> acc_vertex_tag = mesh.create_accessor(vertex_tag_hanlde);
    Accessor<long> acc_todo_tag = mesh.create_accessor(todo_handle);
    for (const Tuple& t : mesh.get_all(PrimitiveType::Edge)) {
        if (acc_vertex_tag.scalar_attribute(t) == 1 &&
            acc_vertex_tag.scalar_attribute(mesh.switch_vertex(t)) == 2) {
            acc_todo_tag.scalar_attribute(t) = 1;
        } else if (
            acc_vertex_tag.scalar_attribute(t) == 2 &&
            acc_vertex_tag.scalar_attribute(mesh.switch_vertex(t)) == 1) {
            acc_todo_tag.scalar_attribute(t) = 1;
        } else {
            acc_todo_tag.scalar_attribute(t) = 0;
        }
    }

    operations::OperationSettings<operations::tri_mesh::VertexPushOffset> settings;
    settings.edge_tag_handle = edge_tag_handle;
    settings.embedding_tag_value = 0;
    settings.offset_tag_value = 2;
    settings.input_tag_value = 1;
    settings.offset_len = 5;
    settings.position = pos_handle;
    settings.vertex_tag_handle = vertex_tag_hanlde;
    settings.todo_tag_handle = todo_handle;
    settings.initialize_invariants(mesh);
    Scheduler scheduler(mesh);

    scheduler.add_operation_type<operations::tri_mesh::VertexPushOffset>("vertex_push", settings);
    while (true) {
        scheduler.run_operation_on_all(PrimitiveType::Edge, "vertex_push");
        if (scheduler.number_of_successful_operations() == 0) {
            break;
        }
    }

    if (true) {
        ParaviewWriter
            writer(data_dir / "2Dpipeline_result", "position", mesh, true, true, true, false);
        mesh.serialize(writer);
    }
}