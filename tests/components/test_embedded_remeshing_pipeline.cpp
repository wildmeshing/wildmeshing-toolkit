#include <stdio.h>
#include <stdlib.h>
#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <nlohmann/json.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/operations/tet_mesh/EdgeSplit.hpp>
#include <wmtk/operations/tet_mesh/VertexLaplacianSmoothWithTags.hpp>
#include <wmtk/operations/tet_mesh/VertexPushOffset.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplit.hpp>
#include <wmtk/operations/tri_mesh/VertexLaplacianSmoothWithTags.hpp>
#include <wmtk/operations/tri_mesh/VertexPushOffset.hpp>
#include <wmtk/operations/utils/HelperFunctions.hpp>
#include <wmtk/simplex/link.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk_components/embedded_remeshing/internal/EmbeddedRemeshing.hpp>
#include <wmtk_components/embedded_remeshing/internal/ModelLoader.hpp>
#include <wmtk_components/input/input.hpp>
#include <wmtk_components/input/internal/mesh_with_tag_from_image.hpp>
#include <wmtk_components/marching/internal/Marching.hpp>
#include <wmtk_components/mesh_info/mesh_info.hpp>
#include <wmtk_components/regular_space/internal/RegularSpace.hpp>
#include <wmtk_components/regular_space/regular_space.hpp>
#include <wmtk_components/taubin_smoothing_within_scalffold/internal/TaubinSmoothingWithinScalffold.hpp>
#include "wmtk/../../tests/tools/DEBUG_TetMesh.hpp"
#include "wmtk/../../tests/tools/DEBUG_TriMesh.hpp"
#include "wmtk/../../tests/tools/TetMesh_examples.hpp"
#include "wmtk/../../tests/tools/TriMesh_examples.hpp"

using json = nlohmann::json;
using namespace wmtk;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("embedded_remeshing_2D_pipeline", "[pipeline][2D][.]")
{
    // std::vector<std::vector<long>> labels;
    // for (long j = 0; j < 22; ++j) {
    //     std::vector<long> line;
    //     line.reserve(20);
    //     for (long i = 0; i < 20; ++i) {
    //         if ((i - 10) * (i - 10) + (j - 11) * (j - 11) < 36) {
    //             line.push_back(1);
    //         } else {
    //             line.push_back(0);
    //         }
    //     }
    //     labels.push_back(line);
    // }
    // TriMesh mesh;
    // wmtk::components::internal::load_matrix_in_trimesh(mesh, labels);

    using namespace wmtk;
    wmtk::io::Cache cache("wmtk_cache", std::filesystem::current_path());
    std::filesystem::path img_path = data_dir / "images/test_pipeline.png";
    const std::string tag_name = "img_tag";
    std::shared_ptr<TriMesh> m;
    REQUIRE_NOTHROW(m = components::internal::mesh_with_tag_from_image(img_path, tag_name));
    TriMesh& mesh = static_cast<TriMesh&>(*m);

    MeshAttributeHandle<double> pos_handle =
        mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    MeshAttributeHandle<long> vertex_tag_handle =
        mesh.register_attribute<long>("vertex_tag", PrimitiveType::Vertex, 1);
    MeshAttributeHandle<long> edge_tag_handle =
        mesh.register_attribute<long>("edge_tag", PrimitiveType::Edge, 1);
    MeshAttributeHandle<long> face_tag_handle =
        mesh.get_attribute_handle<long>("img_tag", PrimitiveType::Face);
    Accessor<long> acc_face_tag = mesh.create_accessor(face_tag_handle);
    Accessor<long> acc_vertex_tag = mesh.create_accessor(vertex_tag_handle);
    Accessor<long> acc_edge_tag = mesh.create_accessor(edge_tag_handle);
    // load input edge and vertex label
    {
        for (const Tuple& e : mesh.get_all(PrimitiveType::Edge)) {
            if (mesh.is_boundary(e)) {
                continue;
            } else if (
                acc_face_tag.scalar_attribute(e) !=
                acc_face_tag.scalar_attribute(mesh.switch_face(e))) {
                acc_edge_tag.scalar_attribute(e) = 1;
                acc_vertex_tag.scalar_attribute(e) = 1;
                acc_vertex_tag.scalar_attribute(mesh.switch_vertex(e)) = 1;
            }
        }
    }

    MeshAttributeHandle<long> todo_handle_vertex =
        mesh.register_attribute<long>("todo_tag_vertex", PrimitiveType::Vertex, 1, false, 1);
    MeshAttributeHandle<double> laplacian_vector_handle =
        mesh.register_attribute<double>("laplacian", PrimitiveType::Vertex, 3);

    // preprocess the input
    wmtk::components::internal::TaubinSmoothingWithinScalffold taubin(
        pos_handle,
        laplacian_vector_handle,
        vertex_tag_handle,
        todo_handle_vertex,
        1,
        0,
        0.330,
        -0.331);
    taubin.process(mesh, 5);

    {
        ParaviewWriter
            writer(data_dir / "2d_input_save", "position", mesh, true, true, true, false);
        mesh.serialize(writer);

        HDF5Writer hdfwriter(data_dir / ("2d_input_save.hdf5"));
        mesh.serialize(hdfwriter);
    }

    // MeshAttributeHandle<double> pos_handle =
    //     mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    // MeshAttributeHandle<long> vertex_tag_handle =
    //     mesh.get_attribute_handle<long>("vertex_tag", PrimitiveType::Vertex);
    // MeshAttributeHandle<long> edge_tag_handle =
    //     mesh.get_attribute_handle<long>("edge_tag", PrimitiveType::Edge);
    // MeshAttributeHandle<long> face_tag_handle =
    //     mesh.get_attribute_handle<long>("face_tag", PrimitiveType::Face);

    components::internal::RegularSpace rs(pos_handle, vertex_tag_handle, edge_tag_handle, 1, 0, 2);
    rs.process_edge_simplicity_in_2d(mesh);

    MeshAttributeHandle<long> todo_handle_edge =
        mesh.register_attribute<long>("todo_tag_edge", PrimitiveType::Edge, 1, false, 1);

    components::internal::Marching mc(
        pos_handle,
        vertex_tag_handle,
        edge_tag_handle,
        face_tag_handle,
        todo_handle_edge,
        1,
        0,
        2);
    mc.process(mesh);

    {
        ParaviewWriter
            writer(data_dir / "2d_first_stage_save", "position", mesh, true, true, true, false);
        mesh.serialize(writer);

        HDF5Writer hdfwriter(data_dir / ("2d_first_stage_save.hdf5"));
        mesh.serialize(hdfwriter);
    }

    // int iteration_time = 1;
    // wmtk::components::internal::EmbeddedRemeshing er(
    //     vertex_tag_handle,
    //     edge_tag_handle,
    //     face_tag_handle,
    //     todo_handle_vertex,
    //     pos_handle,
    //     1,
    //     0,
    //     2,
    //     5,
    //     1,
    //     true);
    // er.remeshing(mesh, iteration_time);

    if (true) {
        ParaviewWriter
            writer(data_dir / "2Dpipeline_result", "position", mesh, true, true, true, false);
        mesh.serialize(writer);


        HDF5Writer hdfwriter(data_dir / ("2d_save.hdf5"));
        mesh.serialize(hdfwriter);
    }
}

TEST_CASE("test_manifold", "[.]")
{
    std::shared_ptr<Mesh> pre_mesh_in = read_mesh(data_dir / ("2d_first_stage_save.hdf5"));

    TriMesh& pre_mesh = static_cast<TriMesh&>(*pre_mesh_in);

    std::shared_ptr<Mesh> post_mesh_in = read_mesh(data_dir / ("2d_save.hdf5"));

    TriMesh& post_mesh = static_cast<TriMesh&>(*post_mesh_in);

    MeshAttributeHandle<long> pre_vertex_tag_handle =
        pre_mesh.get_attribute_handle<long>("vertex_tag", PrimitiveType::Vertex);
    MeshAttributeHandle<long> pre_edge_tag_handle =
        pre_mesh.get_attribute_handle<long>("edge_tag", PrimitiveType::Edge);
    MeshAttributeHandle<long> post_vertex_tag_handle =
        post_mesh.get_attribute_handle<long>("vertex_tag", PrimitiveType::Vertex);
    MeshAttributeHandle<long> post_edge_tag_handle =
        post_mesh.get_attribute_handle<long>("edge_tag", PrimitiveType::Edge);

    const std::vector<Tuple>& pre_vertices = pre_mesh.get_all(PrimitiveType::Vertex);
    const std::vector<Tuple>& post_vertices = post_mesh.get_all(PrimitiveType::Vertex);
    const std::vector<Tuple>& pre_edges = pre_mesh.get_all(PrimitiveType::Edge);
    const std::vector<Tuple>& post_edges = post_mesh.get_all(PrimitiveType::Edge);

    Accessor<long> acc_pre_vertex = pre_mesh.create_accessor(pre_vertex_tag_handle);
    Accessor<long> acc_post_vertex = pre_mesh.create_accessor(post_vertex_tag_handle);
    Accessor<long> acc_pre_edge = pre_mesh.create_accessor(pre_edge_tag_handle);
    Accessor<long> acc_post_edge = pre_mesh.create_accessor(post_edge_tag_handle);

    CHECK(
        pre_mesh.get_all(PrimitiveType::Vertex).size() ==
        post_mesh.get_all(PrimitiveType::Vertex).size());
    CHECK(
        pre_mesh.get_all(PrimitiveType::Edge).size() ==
        post_mesh.get_all(PrimitiveType::Edge).size());
    long size1 = pre_mesh.get_all(PrimitiveType::Vertex).size();
    long size2 = pre_mesh.get_all(PrimitiveType::Edge).size();
    for (long i = 0; i < size1; ++i) {
        CHECK(
            acc_pre_vertex.scalar_attribute(pre_vertices[i]) ==
            acc_post_vertex.scalar_attribute(post_vertices[i]));
    }
    for (long i = 0; i < size2; ++i) {
        CHECK(
            acc_pre_edge.scalar_attribute(pre_edges[i]) ==
            acc_post_edge.scalar_attribute(post_edges[i]));
    }
}

TEST_CASE("state_continue2D", "[.]")
{
    std::shared_ptr<Mesh> mesh_in = read_mesh(data_dir / ("2d_save.hdf5"));

    TriMesh& mesh = static_cast<TriMesh&>(*mesh_in);

    MeshAttributeHandle<double> pos_handle =
        mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    MeshAttributeHandle<long> vertex_tag_handle =
        mesh.get_attribute_handle<long>("vertex_tag", PrimitiveType::Vertex);
    MeshAttributeHandle<long> edge_tag_handle =
        mesh.get_attribute_handle<long>("edge_tag", PrimitiveType::Edge);
    MeshAttributeHandle<long> face_tag_handle =
        mesh.get_attribute_handle<long>("face_tag", PrimitiveType::Face);
    MeshAttributeHandle<long> todo_handle_edge =
        mesh.get_attribute_handle<long>("todo_tag_edge", PrimitiveType::Edge);
    MeshAttributeHandle<long> todo_handle_vertex =
        mesh.get_attribute_handle<long>("todo_tag_vertex", PrimitiveType::Vertex);

    wmtk::components::internal::EmbeddedRemeshing er(
        vertex_tag_handle,
        edge_tag_handle,
        face_tag_handle,
        todo_handle_vertex,
        pos_handle,
        1,
        0,
        2,
        5.0,
        1.0,
        true);
    er.remeshing(mesh, 1);

    if (true) {
        ParaviewWriter
            writer(data_dir / "2Dpipeline_result", "position", mesh, true, true, true, false);
        mesh.serialize(writer);

        HDF5Writer hdfwriter(data_dir / ("2d_save.hdf5"));
        mesh.serialize(hdfwriter);
    }
}

TEST_CASE("embedded_remeshing_2D_pipeline_for_test", "[pipeline][2D]")
{
    std::vector<std::vector<long>> labels;
    for (long j = 0; j < 5; ++j) {
        std::vector<long> line;
        line.reserve(5);
        for (long i = 0; i < 5; ++i) {
            if ((i - 2) * (i - 2) + (j - 2) * (j - 2) < 1) {
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
    MeshAttributeHandle<long> vertex_tag_handle =
        mesh.get_attribute_handle<long>("vertex_tag", PrimitiveType::Vertex);
    MeshAttributeHandle<long> edge_tag_handle =
        mesh.get_attribute_handle<long>("edge_tag", PrimitiveType::Edge);
    MeshAttributeHandle<long> face_tag_handle =
        mesh.get_attribute_handle<long>("face_tag", PrimitiveType::Face);
    Accessor<long> acc_face_tag = mesh.create_accessor(face_tag_handle);
    Accessor<long> acc_vertex_tag = mesh.create_accessor(vertex_tag_handle);
    Accessor<long> acc_edge_tag = mesh.create_accessor(edge_tag_handle);
    // load input edge and vertex label
    {
        for (const Tuple& e : mesh.get_all(PrimitiveType::Edge)) {
            if (mesh.is_boundary(e)) {
                continue;
            } else if (
                acc_face_tag.scalar_attribute(e) !=
                acc_face_tag.scalar_attribute(mesh.switch_face(e))) {
                acc_edge_tag.scalar_attribute(e) = 1;
                acc_vertex_tag.scalar_attribute(e) = 1;
                acc_vertex_tag.scalar_attribute(mesh.switch_vertex(e)) = 1;
            }
        }
    }

    {
        ParaviewWriter
            writer(data_dir / "2d_input_save", "position", mesh, true, true, true, false);
        mesh.serialize(writer);

        HDF5Writer hdfwriter(data_dir / ("2d_input_save.hdf5"));
        mesh.serialize(hdfwriter);
    }

    // MeshAttributeHandle<double> pos_handle =
    //     mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    // MeshAttributeHandle<long> vertex_tag_handle =
    //     mesh.get_attribute_handle<long>("vertex_tag", PrimitiveType::Vertex);
    // MeshAttributeHandle<long> edge_tag_handle =
    //     mesh.get_attribute_handle<long>("edge_tag", PrimitiveType::Edge);
    // MeshAttributeHandle<long> face_tag_handle =
    //     mesh.get_attribute_handle<long>("face_tag", PrimitiveType::Face);

    components::internal::RegularSpace rs(pos_handle, vertex_tag_handle, edge_tag_handle, 1, 0, 2);
    rs.process_edge_simplicity_in_2d(mesh);

    MeshAttributeHandle<long> todo_handle_edge =
        mesh.register_attribute<long>("todo_tag_edge", PrimitiveType::Edge, 1, false, 1);

    MeshAttributeHandle<long> todo_handle_vertex =
        mesh.register_attribute<long>("todo_tag_vertex", PrimitiveType::Vertex, 1, false, 1);

    components::internal::Marching mc(
        pos_handle,
        vertex_tag_handle,
        edge_tag_handle,
        face_tag_handle,
        todo_handle_edge,
        1,
        0,
        2);
    mc.process(mesh);

    {
        ParaviewWriter
            writer(data_dir / "2d_first_stage_save", "position", mesh, true, true, true, false);
        mesh.serialize(writer);

        HDF5Writer hdfwriter(data_dir / ("2d_first_stage_save.hdf5"));
        mesh.serialize(hdfwriter);
    }

    int iteration_time = 1;
    wmtk::components::internal::EmbeddedRemeshing er(
        vertex_tag_handle,
        edge_tag_handle,
        face_tag_handle,
        todo_handle_vertex,
        pos_handle,
        1,
        0,
        2,
        5,
        1,
        true);
    er.remeshing(mesh, iteration_time);

    if (true) {
        ParaviewWriter
            writer(data_dir / "2Dpipeline_result", "position", mesh, true, true, true, false);
        mesh.serialize(writer);


        HDF5Writer hdfwriter(data_dir / ("2d_save.hdf5"));
        mesh.serialize(hdfwriter);
    }
}

TEST_CASE("embedded_remeshing_3D_pipeline", "[pipeline][3D]")
{
    long dim_len = 10;
    long r = 5;
    std::vector<std::vector<std::vector<long>>> labels;
    for (long k = 0; k < dim_len; ++k) {
        std::vector<std::vector<long>> layer;
        for (long j = 0; j < dim_len; ++j) {
            std::vector<long> line;
            line.reserve(dim_len);
            for (long i = 0; i < dim_len; ++i) {
                if ((i - dim_len / 2) * (i - dim_len / 2) + (j - dim_len / 2) * (j - dim_len / 2) +
                        (k - dim_len / 2) * (k - dim_len / 2) <
                    r * r) {
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


    MeshAttributeHandle<long> todo_handle_vertex =
        mesh.register_attribute<long>("todo_tag_vertex", PrimitiveType::Vertex, 1, false, 1);
    MeshAttributeHandle<double> laplacian_vector_handle =
        mesh.register_attribute<double>("laplacian", PrimitiveType::Vertex, 3);
    MeshAttributeHandle<double> pos_handle =
        mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    MeshAttributeHandle<long> vertex_tag_handle =
        mesh.get_attribute_handle<long>("vertex_tag", PrimitiveType::Vertex);

    // preprocess the input
    wmtk::components::internal::TaubinSmoothingWithinScalffold taubin(
        pos_handle,
        laplacian_vector_handle,
        vertex_tag_handle,
        todo_handle_vertex,
        1,
        0,
        0.330,
        -0.331);
    taubin.process(mesh, 5);

    MeshAttributeHandle<long> edge_tag_handle =
        mesh.get_attribute_handle<long>("edge_tag", PrimitiveType::Edge);
    MeshAttributeHandle<long> face_tag_handle =
        mesh.get_attribute_handle<long>("face_tag", PrimitiveType::Face);

    components::internal::RegularSpace rs(pos_handle, vertex_tag_handle, edge_tag_handle, 1, 0, 2);
    rs.process_face_simplicity_in_3d(mesh, face_tag_handle);

    MeshAttributeHandle<long> todo_handle_edge =
        mesh.register_attribute<long>("todo_tag_edge", PrimitiveType::Edge, 1, false, 1);

    components::internal::Marching mc(
        pos_handle,
        vertex_tag_handle,
        edge_tag_handle,
        face_tag_handle,
        todo_handle_edge,
        1,
        0,
        2);
    mc.process(mesh);

    {
        HDF5Writer hdfwriter(data_dir / ("save_first_stage.hdf5"));
        mesh.serialize(hdfwriter);
    }

    int iteration_time = 1;
    wmtk::components::internal::EmbeddedRemeshing er(
        vertex_tag_handle,
        edge_tag_handle,
        face_tag_handle,
        todo_handle_vertex,
        pos_handle,
        1,
        0,
        2,
        5,
        1,
        true);
    er.remeshing(mesh, iteration_time);

    if (true) {
        ParaviewWriter
            writer(data_dir / "3Dpipeline_result", "position", mesh, true, true, true, true);
        mesh.serialize(writer);

        HDF5Writer hdfwriter(data_dir / ("save.hdf5"));
        mesh.serialize(hdfwriter);
    }
}

TEST_CASE("state_continue3D", "[.]")
{
    std::shared_ptr<Mesh> mesh_in = read_mesh(data_dir / ("save.hdf5"));

    TetMesh& mesh = static_cast<TetMesh&>(*mesh_in);

    MeshAttributeHandle<double> pos_handle =
        mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    MeshAttributeHandle<long> vertex_tag_handle =
        mesh.get_attribute_handle<long>("vertex_tag", PrimitiveType::Vertex);
    MeshAttributeHandle<long> edge_tag_handle =
        mesh.get_attribute_handle<long>("edge_tag", PrimitiveType::Edge);
    MeshAttributeHandle<long> face_tag_handle =
        mesh.get_attribute_handle<long>("face_tag", PrimitiveType::Face);
    MeshAttributeHandle<long> todo_handle_edge =
        mesh.get_attribute_handle<long>("todo_tag_edge", PrimitiveType::Edge);
    MeshAttributeHandle<long> todo_handle_vertex =
        mesh.get_attribute_handle<long>("todo_tag_vertex", PrimitiveType::Vertex);

    int iteration_time = 1;
    wmtk::components::internal::EmbeddedRemeshing er(
        vertex_tag_handle,
        edge_tag_handle,
        face_tag_handle,
        todo_handle_vertex,
        pos_handle,
        1,
        0,
        2,
        5.0,
        1.0,
        true);
    er.remeshing(mesh, iteration_time);

    if (true) {
        ParaviewWriter
            writer(data_dir / "3Dpipeline_result", "position", mesh, true, true, true, true);
        mesh.serialize(writer);

        HDF5Writer hdfwriter(data_dir / ("save.hdf5"));
        mesh.serialize(hdfwriter);
    }
}

TEST_CASE("matrix_load", "[pipeline]")
{
    SECTION("trimesh_load")
    {
        std::vector<std::vector<long>> labels;
        for (long j = 0; j < 11; ++j) {
            std::vector<long> line;
            line.reserve(10);
            for (long i = 0; i < 10; ++i) {
                if ((i - 5) * (i - 5) + (j - 5) * (j - 5) < 9) {
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
        for (long k = 0; k < 12; ++k) {
            std::vector<std::vector<long>> layer;
            for (long j = 0; j < 11; ++j) {
                std::vector<long> line;
                line.reserve(10);
                for (long i = 0; i < 10; ++i) {
                    if ((i - 5) * (i - 5) + (j - 5) * (j - 5) + (k - 6) * (k - 6) < 9) {
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
        if (false) {
            ParaviewWriter
                writer(data_dir / "tetmesh_matrix_load", "position", mesh, true, true, true, true);
            mesh.serialize(writer);
        }
    }
}

TEST_CASE("embedded_remeshing_relocation_helper_function_invert", "[pipeline][helper_function]")
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

TEST_CASE("embedded_remeshing_relocation_helper_function_push", "[pipeline][helper_function]")
{
    SECTION("helper_function_push")
    {
        //    0---1---2
        //   /0\1/2\3/4\ .
        //  3---4---5---6
        //   \5/6\7/  .
        //    7---8
        //
        //    0---1---2
        //   / \ / \ / \ .
        //  3---4---5---6
        //   \ / \ / \ /
        //    7---8---9
        tests::DEBUG_TriMesh m = wmtk::tests::edge_region_with_position();
        MeshAttributeHandle<long> edge_tag_handle =
            m.register_attribute<long>("edge_tag", PrimitiveType::Edge, 1);
        MeshAttributeHandle<long> vertex_tag_handle =
            m.register_attribute<long>("vertex_tag", PrimitiveType::Vertex, 1);
        MeshAttributeHandle<long> todo_tag_handle =
            m.register_attribute<long>("todo_tag", PrimitiveType::Vertex, 1, false, 1);
        Accessor<long> acc_edge_tag = m.create_accessor(edge_tag_handle);
        Accessor<long> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
        acc_vertex_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[3]) = 1;
        acc_vertex_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[7]) = 1;
        acc_vertex_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[8]) = 1;
        acc_vertex_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[4]) = 2;
        acc_vertex_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[5]) = 2;
        acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(3, 7, 5)) = 1;
        acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(7, 8, 6)) = 1;

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
        operations::tri_mesh::VertexPushOffset op1(
            m,
            m.get_all(PrimitiveType::Vertex)[4],
            settings);
        CHECK(op1());
        operations::tri_mesh::VertexPushOffset op2(
            m,
            m.get_all(PrimitiveType::Vertex)[5],
            settings);
        CHECK(op2());
        if (false) {
            ParaviewWriter writer(data_dir / "push_result", "position", m, true, true, true, false);
            m.serialize(writer);
        }
    }
}

TEST_CASE("helper_function_is_invert", "[helper_function]")
{
    {
        using namespace tests_3d;
        //        0 ---------- 4
        //       / \\        // \ .
        //      /   \ \     //   \ .
        //     /     \  \  //     \ .
        //    /       \   \3       \ .
        //  1 --------- 2/ -------- 5   tuple edge 2-3
        //    \       /  /\ \      / .
        //     \     / /   \\     / .
        //      \   //      \\   / .
        //       \ //        \  / .
        //        6 -----------7
        //
        DEBUG_TetMesh m = six_cycle_tets();
        const long embedding_tag_value = 0;
        const long input_tag_value = 1;
        const long split_tag_value = 2;
        Eigen::MatrixXd V(8, 3);
        V.row(0) << 0.5, 0.86, 0;
        V.row(1) << 0, 0, 0;
        V.row(2) << 1.0, 0, 1.0;
        V.row(3) << 1.0, 0, -1.0;
        V.row(4) << 1.5, 0.86, 0;
        V.row(5) << 2, 0, 0;
        V.row(6) << 0.5, -0.86, 0;
        V.row(7) << 1.5, -0.86, 0;
        mesh_utils::set_matrix_attribute(V, "position", PrimitiveType::Vertex, m);
        MeshAttributeHandle<double> pos_handle =
            m.get_attribute_handle<double>("position", wmtk::PrimitiveType::Vertex);

        CHECK(!wmtk::operations::utils::is_invert(
            m,
            pos_handle,
            m.get_all(PrimitiveType::Vertex)[2],
            PrimitiveType::Tetrahedron,
            Eigen::Vector3d(1.0, 0, 1.0)));
    }
    {
        TetMesh mesh = wmtk::tests_3d::single_tet(); // wmtk::tests::two_neighbors_plus_one();
        Eigen::MatrixXd V(5, 3);
        V.row(0) << 0, 0, 0;
        V.row(1) << -1, -1, 0;
        V.row(2) << 0.5, 0.86, 0;
        V.row(3) << 0.5, 0.43, 1;
        MeshAttributeHandle<double> pos_handle =
            mesh_utils::set_matrix_attribute(V, "position", PrimitiveType::Vertex, mesh);

        CHECK(wmtk::operations::utils::is_invert(
            mesh,
            pos_handle,
            mesh.get_all(PrimitiveType::Vertex)[1],
            PrimitiveType::Tetrahedron,
            Eigen::Vector3d(1.0, 0, 0)));
    }
}

TEST_CASE("helper_function_nearest_point", "[helper_function]")
{
    SECTION("2D_case")
    {
        using namespace tests;
        const long embedding_tag_value = 0;
        const long input_tag_value = 1;
        const long split_tag_value = 2;
        DEBUG_TriMesh m = wmtk::tests::single_equilateral_triangle();
        MeshAttributeHandle<double> pos_handle =
            m.get_attribute_handle<double>("position", wmtk::PrimitiveType::Vertex);
        Accessor<double> acc_pos = m.create_accessor(pos_handle);
        Eigen::Vector3d p = wmtk::operations::utils::nearest_point_to_edge(
            m,
            pos_handle,
            m.edge_tuple_between_v1_v2(1, 2, 0),
            m.get_all(PrimitiveType::Vertex)[0]);
        spdlog::info("{},{},{}", p.x(), p.y(), p.z());
    }
    SECTION("3D_case")
    {
        using namespace tests_3d;
        //        0
        //       / \\ .
        //      /   \ \ .
        //     /     \  \ .
        //    /       \   \ 3
        //  1 --------- 2
        //
        const long embedding_tag_value = 0;
        const long input_tag_value = 1;
        const long split_tag_value = 2;
        DEBUG_TetMesh m = wmtk::tests_3d::single_tet(); // wmtk::tests::two_neighbors_plus_one();
        Eigen::MatrixXd V(5, 3);
        V.row(0) << -1.0, 0.5, 0.9;
        V.row(1) << 0, 0, 0;
        V.row(2) << 1.0, 0, 0;
        V.row(3) << 0.0, 1.5, 0.0;
        mesh_utils::set_matrix_attribute(V, "position", PrimitiveType::Vertex, m);
        MeshAttributeHandle<double> pos_handle =
            m.get_attribute_handle<double>("position", wmtk::PrimitiveType::Vertex);
        Accessor<double> acc_pos = m.create_accessor(pos_handle);
        Eigen::Vector3d p = wmtk::operations::utils::nearest_point_to_face(
            m,
            pos_handle,
            m.face_tuple_from_vids(1, 2, 3),
            m.get_all(PrimitiveType::Vertex)[0]);
        spdlog::info("{},{},{}", p.x(), p.y(), p.z());
    }
}

TEST_CASE("tet_embedded_remeshing_smoothing_operation", "[embedded_remeshing][operation]")
{
    using namespace tests_3d;
    //        0 ---------- 4
    //       / \\        // \ .
    //      /   \ \     //   \ .
    //     /     \  \  //     \ .
    //    /       \   \3       \ .
    //  1 --------- 2/ -------- 5   tuple edge 2-3
    //    \       /  /\ \      / .
    //     \     / /   \\     / .
    //      \   //      \\   / .
    //       \ //        \  / .
    //        6 -----------7
    //
    DEBUG_TetMesh m = six_cycle_tets();
    const long embedding_tag_value = 0;
    const long input_tag_value = 1;
    const long split_tag_value = 2;
    Eigen::MatrixXd V(8, 3);
    V.row(0) << 0.5, 0.86, 0;
    V.row(1) << 0, 0, 0;
    V.row(2) << 1.0, 0, 1.0;
    V.row(3) << 1.0, 0, -1.0;
    V.row(4) << 1.5, 0.86, 0;
    V.row(5) << 2, 0, 0;
    V.row(6) << 0.5, -0.86, 0;
    V.row(7) << 1.5, -0.86, 0;
    mesh_utils::set_matrix_attribute(V, "position", PrimitiveType::Vertex, m);
    MeshAttributeHandle<double> pos_handle =
        m.get_attribute_handle<double>("position", wmtk::PrimitiveType::Vertex);
    Accessor<double> acc_pos = m.create_accessor(pos_handle);
    MeshAttributeHandle<long> edge_tag_handle =
        m.register_attribute<long>("edge_tag", PrimitiveType::Edge, 1);
    MeshAttributeHandle<long> vertex_tag_handle =
        m.register_attribute<long>("vertex_tag", PrimitiveType::Vertex, 1, false, 0);
    MeshAttributeHandle<long> todo_handle_vertex =
        m.register_attribute<long>("todo_tag", PrimitiveType::Vertex, 1, false, 1);
    Accessor<long> acc_edge_tag = m.create_accessor(edge_tag_handle);
    Accessor<long> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
    Accessor<long> acc_todo_tag = m.create_accessor(todo_handle_vertex);
    acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(1, 2, 0)) = 2;
    acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(0, 2, 0)) = 2;
    acc_vertex_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[0]) = 2;
    acc_vertex_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[1]) = 2;
    acc_vertex_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[2]) = 2;

    operations::OperationSettings<operations::tet_mesh::EdgeSplit> splitsetting;
    splitsetting.initialize_invariants(m);
    operations::tet_mesh::EdgeSplit split_op(m, m.edge_tuple_between_v1_v2(2, 3, 0), splitsetting);
    CHECK(split_op());
    acc_pos.vector_attribute(m.get_all(PrimitiveType::Vertex)[8]) << 1, 0, 0;

    operations::OperationSettings<operations::tet_mesh::VertexLaplacianSmoothWithTags> settings;
    settings.edge_tag_handle = edge_tag_handle;
    settings.embedding_tag_value = 0;
    settings.offset_tag_value = 2;
    settings.position = pos_handle;
    settings.todo_tag_handle = todo_handle_vertex;
    settings.vertex_tag_handle = vertex_tag_handle;
    settings.initialize_invariants(m);
    operations::tet_mesh::VertexLaplacianSmoothWithTags op1(
        m,
        m.get_all(PrimitiveType::Vertex)[8],
        settings);
    acc_todo_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[8]) = 1;
    CHECK(op1());
    Eigen::Vector3d p = acc_pos.vector_attribute(m.get_all(PrimitiveType::Vertex)[8]);
    spdlog::info("{},{},{}", p.x(), p.y(), p.z());

    acc_vertex_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[8]) = 1;
    operations::tet_mesh::VertexLaplacianSmoothWithTags op2(
        m,
        m.get_all(PrimitiveType::Vertex)[8],
        settings);
    acc_todo_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[8]) = 1;
    CHECK_THROWS(op2());

    acc_vertex_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[8]) = 2;
    operations::tet_mesh::VertexLaplacianSmoothWithTags op3(
        m,
        m.get_all(PrimitiveType::Vertex)[8],
        settings);
    acc_todo_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[8]) = 1;
    CHECK_THROWS(op3());
}

TEST_CASE("tri_embedded_remeshing_smoothing_operation", "[embedded_remeshing][operation]")
{
    //    0---1---2
    //   /0\1/2\3/4\ .
    //  3---4---5---6
    //   \5/6\7/  .
    //    7---8
    //
    //    0---1---2
    //   / \ / \ / \ .
    //  3---4---5---6
    //   \ / \ / \ /
    //    7---8---9
    tests::DEBUG_TriMesh m = wmtk::tests::edge_region_with_position();
    MeshAttributeHandle<long> edge_tag_handle =
        m.register_attribute<long>("edge_tag", PrimitiveType::Edge, 1);
    MeshAttributeHandle<long> vertex_tag_handle =
        m.register_attribute<long>("vertex_tag", PrimitiveType::Vertex, 1);
    MeshAttributeHandle<long> todo_tag_handle =
        m.register_attribute<long>("todo_tag", PrimitiveType::Vertex, 1, false, 1);
    Accessor<long> acc_edge_tag = m.create_accessor(edge_tag_handle);
    Accessor<long> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
    acc_vertex_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[3]) = 1;
    acc_vertex_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[7]) = 1;
    acc_vertex_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[8]) = 1;
    acc_vertex_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[3]) = 2;
    acc_vertex_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[4]) = 2;
    acc_vertex_tag.scalar_attribute(m.get_all(PrimitiveType::Vertex)[5]) = 2;
    acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(3, 4, 0)) = 2;
    acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(4, 5, 0)) = 2;

    operations::OperationSettings<operations::tri_mesh::VertexLaplacianSmoothWithTags> settings;
    settings.edge_tag_handle = edge_tag_handle;
    settings.embedding_tag_value = 0;
    settings.offset_tag_value = 2;
    settings.position = m.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    settings.vertex_tag_handle = vertex_tag_handle;
    settings.todo_tag_handle = todo_tag_handle;
    settings.initialize_invariants(m);
    operations::tri_mesh::VertexLaplacianSmoothWithTags op1(
        m,
        m.get_all(PrimitiveType::Vertex)[4],
        settings);
    CHECK(op1());
    operations::tri_mesh::VertexLaplacianSmoothWithTags op2(
        m,
        m.get_all(PrimitiveType::Vertex)[5],
        settings);
    CHECK_THROWS(op2());
    if (false) {
        ParaviewWriter writer(data_dir / "push_result", "position", m, true, true, true, false);
        m.serialize(writer);
    }
}

TEST_CASE("test_push_point", "[test][.]")
{
    using namespace tests_3d;
    //        0 ---------- 4
    //       / \\        // \ .
    //      /   \ \     //   \ .
    //     /     \  \  //     \ .
    //    /       \   \3       \ .
    //  1 --------- 2/ -------- 5   tuple edge 2-3
    //    \       /  /\ \      / .
    //     \     / /   \\     / .
    //      \   //      \\   / .
    //       \ //        \  / .
    //        6 -----------7
    //
    DEBUG_TetMesh m = six_cycle_tets();
    const long embedding_tag_value = 0;
    const long input_tag_value = 1;
    const long split_tag_value = 2;
    Eigen::MatrixXd V(8, 3);
    V.row(0) << 0.5, 0.86, 0;
    V.row(1) << 0, 0, 0;
    V.row(2) << 1.0, 0, 1.0;
    V.row(3) << 1.0, 0, -1.0;
    V.row(4) << 1.5, 0.86, 0;
    V.row(5) << 2, 0, 0;
    V.row(6) << 0.5, -0.86, 0;
    V.row(7) << 1.5, -0.86, 0;
    mesh_utils::set_matrix_attribute(V, "position", PrimitiveType::Vertex, m);
    MeshAttributeHandle<double> pos_handle =
        m.get_attribute_handle<double>("position", wmtk::PrimitiveType::Vertex);
    MeshAttributeHandle<long> edge_tag_handle =
        m.register_attribute<long>("edge_tag", PrimitiveType::Edge, 1);
    MeshAttributeHandle<long> vertex_tag_handle =
        m.register_attribute<long>("vertex_tag", PrimitiveType::Vertex, 1);
    MeshAttributeHandle<long> todo_tag_handle =
        m.register_attribute<long>("todo_tag", PrimitiveType::Edge, 1, false, 1);
    MeshAttributeHandle<long> face_tag_handle =
        m.register_attribute<long>("face_tag", PrimitiveType::Face, 1);
    Accessor<double> acc_pos = m.create_accessor(pos_handle);
    Accessor<long> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
    Accessor<long> acc_edge_tag = m.create_accessor(edge_tag_handle);
    Accessor<long> acc_face_tag = m.create_accessor(face_tag_handle);

    acc_face_tag.scalar_attribute(m.face_tuple_from_vids(0, 1, 2)) = 2;
    acc_face_tag.scalar_attribute(m.face_tuple_from_vids(0, 1, 2)) = 2;
    acc_face_tag.scalar_attribute(m.face_tuple_from_vids(0, 1, 2)) = 2;


    operations::OperationSettings<operations::tet_mesh::VertexPushOffset> settings;
    settings.edge_tag_handle = edge_tag_handle;
    settings.embedding_tag_value = 0;
    settings.offset_tag_value = 2;
    settings.input_tag_value = 1;
    settings.offset_len = 2.0;
    settings.position = pos_handle;
    settings.vertex_tag_handle = vertex_tag_handle;
    settings.todo_tag_handle = todo_tag_handle;
    settings.initialize_invariants(m);
}