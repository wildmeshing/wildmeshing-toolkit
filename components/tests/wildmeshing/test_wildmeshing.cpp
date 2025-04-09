#include <catch2/catch_test_macros.hpp>

#include <tools/DEBUG_EdgeMesh.hpp>
#include <tools/DEBUG_TriMesh.hpp>
#include <tools/EdgeMesh_examples.hpp>
#include <tools/TriMesh_examples.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PrimitiveType.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/submesh/Embedding.hpp>
#include <wmtk/submesh/SubMesh.hpp>
#include <wmtk/submesh/utils/submesh_from_multimesh.hpp>
#include <wmtk/submesh/utils/write.hpp>

#include <wmtk/components/wildmeshing/wildmeshing.hpp>


using namespace wmtk;
using namespace components;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Triangle;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("test_wildmeshing_submesh", "[wildmeshing][.]")
{
    // logger().set_level(spdlog::level::off);
    logger().set_level(spdlog::level::debug);
    opt_logger().set_level(spdlog::level::warn);

    std::shared_ptr<tests::DEBUG_TriMesh> mesh_ptr =
        std::make_shared<tests::DEBUG_TriMesh>(tests::edge_region_with_position_2D());
    tests::DEBUG_TriMesh& m = *mesh_ptr;

    // add input
    std::shared_ptr<tests::DEBUG_EdgeMesh> child_input_ptr;
    {
        child_input_ptr = std::make_shared<tests::DEBUG_EdgeMesh>(tests::single_line());

        const Tuple child_tuple = child_input_ptr->edge_tuple_from_vids(0, 1);
        const Tuple parent_tuple = m.edge_tuple_from_vids(4, 5);
        std::vector<std::array<Tuple, 2>> map_tuples;
        map_tuples.emplace_back(std::array<Tuple, 2>{child_tuple, parent_tuple});
        m.register_child_mesh(child_input_ptr, map_tuples);
    }
    // add boundary
    std::shared_ptr<tests::DEBUG_EdgeMesh> child_bnd_ptr;
    {
        child_bnd_ptr = std::make_shared<tests::DEBUG_EdgeMesh>();
        tests::DEBUG_EdgeMesh& c = *child_bnd_ptr;

        RowVectors2l edges;
        edges.resize(8, 2);
        edges.row(0) << 0, 1;
        edges.row(1) << 1, 2;
        edges.row(2) << 2, 3;
        edges.row(3) << 3, 4;
        edges.row(4) << 4, 5;
        edges.row(5) << 5, 6;
        edges.row(6) << 6, 7;
        edges.row(7) << 7, 0;
        c.initialize(edges);

        std::vector<std::array<Tuple, 2>> map_tuples;
        map_tuples.emplace_back(
            std::array<Tuple, 2>{c.edge_tuple_from_vids(0, 1), m.edge_tuple_from_vids(0, 1)});
        map_tuples.emplace_back(
            std::array<Tuple, 2>{c.edge_tuple_from_vids(1, 2), m.edge_tuple_from_vids(1, 2)});
        map_tuples.emplace_back(
            std::array<Tuple, 2>{c.edge_tuple_from_vids(2, 3), m.edge_tuple_from_vids(2, 6)});
        map_tuples.emplace_back(
            std::array<Tuple, 2>{c.edge_tuple_from_vids(3, 4), m.edge_tuple_from_vids(6, 9)});
        map_tuples.emplace_back(
            std::array<Tuple, 2>{c.edge_tuple_from_vids(4, 5), m.edge_tuple_from_vids(9, 8)});
        map_tuples.emplace_back(
            std::array<Tuple, 2>{c.edge_tuple_from_vids(5, 6), m.edge_tuple_from_vids(8, 7)});
        map_tuples.emplace_back(
            std::array<Tuple, 2>{c.edge_tuple_from_vids(6, 7), m.edge_tuple_from_vids(7, 3)});
        map_tuples.emplace_back(
            std::array<Tuple, 2>{c.edge_tuple_from_vids(7, 0), m.edge_tuple_from_vids(3, 0)});
        m.register_child_mesh(child_bnd_ptr, map_tuples);
    }

    // envelopes
    std::vector<wmtk::components::EnvelopeOptions> enves;
    {
        wmtk::components::EnvelopeOptions e;
        e.envelope_name = "input";
        e.envelope_constrained_mesh = child_input_ptr;
        e.envelope_geometry_mesh = child_input_ptr;
        e.thickness = 1e-3;

        enves.push_back(e);

        wmtk::components::EnvelopeOptions e2;
        e2.envelope_name = "bbox";
        e2.envelope_constrained_mesh = child_bnd_ptr;
        e2.envelope_geometry_mesh = child_bnd_ptr;
        e2.thickness = 1e-4;

        enves.push_back(e2);
    }

    wmtk::components::WildMeshingOptions wmo;
    wmo.input_mesh = mesh_ptr;
    wmo.target_edge_length = 0.05;
    wmo.max_passes = 5;
    wmo.replace_double_coordinate = true;
    wmo.intermediate_output_path = "";
    wmo.intermediate_output_name = "out";
    wmo.intermediate_output = true;
    wmo.envelopes = enves;
    wmo.use_embedding = true;

    auto meshes_after_tetwild = wildmeshing(wmo);

    // CHECK(emb.has_child_mesh());
    // CHECK(emb.get_child_meshes().size() == 2);
    //{
    //     using submesh::utils::write;
    //     CHECK_NOTHROW(write("wildmeshing_submesh", "vertices", emb, true, true, true, false));
    // }
}