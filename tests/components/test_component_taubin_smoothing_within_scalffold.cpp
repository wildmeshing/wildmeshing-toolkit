#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <nlohmann/json.hpp>
#include <wmtk/../../tests/tools/DEBUG_TetMesh.hpp>
#include <wmtk/../../tests/tools/DEBUG_TriMesh.hpp>
#include <wmtk/../../tests/tools/TetMesh_examples.hpp>
#include <wmtk/../../tests/tools/TriMesh_examples.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplit.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk_components/embedded_remeshing/internal/ModelLoader.hpp>
#include <wmtk_components/input/internal/mesh_with_tag_from_image.hpp>
#include <wmtk_components/marching/internal/Marching.hpp>
#include <wmtk_components/marching/internal/MarchingOptions.hpp>
#include <wmtk_components/marching/marching.hpp>
#include <wmtk_components/mesh_info/mesh_info.hpp>
#include <wmtk_components/taubin_smoothing_within_scalffold/internal/TaubinSmoothingWithinScalffold.hpp>

using json = nlohmann::json;
using namespace wmtk;

const std::filesystem::path data_dir = WMTK_DATA_DIR;


TEST_CASE("taubin_smoothing_within_scalffold_2D", "[2D][taubin_smoothing][.]")
{
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
    if (true) {
        ParaviewWriter writer(
            data_dir / "2d_before_taubin_smoothing",
            "position",
            mesh,
            true,
            true,
            true,
            false);
        mesh.serialize(writer);
    }

    MeshAttributeHandle<long> todo_handle_vertex =
        mesh.register_attribute<long>("todo_tag_vertex", PrimitiveType::Vertex, 1, false, 0);
    MeshAttributeHandle<double> laplacian_vector_handle =
        mesh.register_attribute<double>("laplacian", PrimitiveType::Vertex, 3);

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

    if (true) {
        ParaviewWriter writer(
            data_dir / "2d_after_taubin_smoothing",
            "position",
            mesh,
            true,
            true,
            true,
            false);
        mesh.serialize(writer);

        HDF5Writer hdfwriter(data_dir / ("taubin_save2d.hdf5"));
        mesh.serialize(hdfwriter);
    }
}

TEST_CASE("continue_taubin_smoothing_within_scalffold_2D", "[2D][taubin_smoothing][.]")
{
    using namespace wmtk;

    std::shared_ptr<Mesh> mesh_in = read_mesh(data_dir / ("taubin_save2d.hdf5"));

    TriMesh& mesh = static_cast<TriMesh&>(*mesh_in);

    MeshAttributeHandle<double> pos_handle =
        mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    MeshAttributeHandle<double> laplacian_vector_handle =
        mesh.get_attribute_handle<double>("laplacian", PrimitiveType::Vertex);
    MeshAttributeHandle<long> vertex_tag_handle =
        mesh.get_attribute_handle<long>("vertex_tag", PrimitiveType::Vertex);
    MeshAttributeHandle<long> todo_handle_vertex =
        mesh.get_attribute_handle<long>("todo_tag_vertex", PrimitiveType::Vertex);

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

    if (true) {
        ParaviewWriter writer(
            data_dir / "3d_after_taubin_smoothing",
            "position",
            mesh,
            true,
            true,
            true,
            false);
        mesh.serialize(writer);

        HDF5Writer hdfwriter(data_dir / ("taubin_save2d.hdf5"));
        mesh.serialize(hdfwriter);
    }
}

TEST_CASE("taubin_smoothing_within_scalffold_3D", "[3D][taubin_smoothing][.]")
{
    using namespace wmtk;
    long dim_len = 12;
    long r = 4;
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

    MeshAttributeHandle<double> pos_handle =
        mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    MeshAttributeHandle<long> vertex_tag_handle =
        mesh.get_attribute_handle<long>("vertex_tag", PrimitiveType::Vertex);
    MeshAttributeHandle<long> edge_tag_handle =
        mesh.get_attribute_handle<long>("edge_tag", PrimitiveType::Edge);
    MeshAttributeHandle<long> face_tag_handle =
        mesh.get_attribute_handle<long>("face_tag", PrimitiveType::Face);
    if (true) {
        ParaviewWriter writer(
            data_dir / "3d_before_taubin_smoothing",
            "position",
            mesh,
            true,
            true,
            true,
            true);
        mesh.serialize(writer);
    }

    MeshAttributeHandle<long> todo_handle_vertex =
        mesh.register_attribute<long>("todo_tag_vertex", PrimitiveType::Vertex, 1);
    MeshAttributeHandle<double> laplacian_vector_handle =
        mesh.register_attribute<double>("laplacian", PrimitiveType::Vertex, 3);

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

    if (true) {
        ParaviewWriter writer(
            data_dir / "3d_after_taubin_smoothing",
            "position",
            mesh,
            true,
            true,
            true,
            true);
        mesh.serialize(writer);

        HDF5Writer hdfwriter(data_dir / ("taubin_save.hdf5"));
        mesh.serialize(hdfwriter);
    }
}

TEST_CASE("continue_taubin_smoothing_within_scalffold_3D", "[3D][taubin_smoothing][.]")
{
    using namespace wmtk;

    std::shared_ptr<Mesh> mesh_in = read_mesh(data_dir / ("taubin_save.hdf5"));

    TetMesh& mesh = static_cast<TetMesh&>(*mesh_in);

    MeshAttributeHandle<double> pos_handle =
        mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    MeshAttributeHandle<double> laplacian_vector_handle =
        mesh.get_attribute_handle<double>("laplacian", PrimitiveType::Vertex);
    MeshAttributeHandle<long> vertex_tag_handle =
        mesh.get_attribute_handle<long>("vertex_tag", PrimitiveType::Vertex);
    MeshAttributeHandle<long> todo_handle_vertex =
        mesh.get_attribute_handle<long>("todo_tag_vertex", PrimitiveType::Vertex);

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

    if (true) {
        ParaviewWriter writer(
            data_dir / "3d_after_taubin_smoothing",
            "position",
            mesh,
            true,
            true,
            true,
            true);
        mesh.serialize(writer);

        HDF5Writer hdfwriter(data_dir / ("taubin_save.hdf5"));
        mesh.serialize(hdfwriter);
    }
}