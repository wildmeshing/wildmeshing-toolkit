#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/simplex/link.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk_components/marching/internal/Marching.hpp>
#include <wmtk_components/marching/internal/MarchingOptions.hpp>
#include <wmtk_components/marching/marching.hpp>
#include <wmtk_components/mesh_info/mesh_info.hpp>
#include "wmtk/../../tests/tools/DEBUG_TriMesh.hpp"
#include "wmtk/../../tests/tools/TriMesh_examples.hpp"

using json = nlohmann::json;
using namespace wmtk;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("marching_file_reading", "[components][marching][.]")
{
    std::map<std::string, std::filesystem::path> files;
    std::map<std::string, long> tags_value;

    json regular_space_jason = {
        {"type", "regular space"},
        {"input", "inputdir"}, /*input dir*/
        {"output", "outputdir"}, /*output dir*/
        {"demension", 1}, /*0 for vertex, 1 for edge, 2 for face, 3 for tet*/
        {"tags_value", tags_value},
        {"split_tag_value"}};

    // TODO
    // upload regular_space result .hdf5 file and use marching_component API
    REQUIRE(false);
}

TEST_CASE("marching_component", "[components][marching]")
{
    const long input_tag_value_0 = 0;
    const long input_tag_value_1 = 1;
    const long isosurface_tag_value = 2;

    //    0---1---2
    //   / \ / \ / \ .
    //  3---4---5---6
    //   \ / \ /  .
    //    7---8
    tests::DEBUG_TriMesh m = wmtk::tests::hex_plus_two_with_position();

    MeshAttributeHandle<long> vertex_tag_handle = m.register_attribute<long>(
        "vertex_tag",
        PrimitiveType::Vertex,
        1,
        false,
        input_tag_value_0);

    std::tuple<MeshAttributeHandle<long>, long, long> vertex_tags =
        std::make_tuple(vertex_tag_handle, input_tag_value_0, input_tag_value_1);

    std::tuple<std::string, long> output_tags = std::make_tuple("vertex_tag", isosurface_tag_value);

    std::vector<std::tuple<MeshAttributeHandle<long>, long>> filter_tag;


    long expected_isosurface_vertex_num = 0;

    SECTION("4")
    {
        const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
        Accessor<long> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
        acc_vertex_tag.scalar_attribute(vertex_tuples[4]) = input_tag_value_1;

        expected_isosurface_vertex_num = 6;
    }
    SECTION("4-5")
    {
        const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
        Accessor<long> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
        acc_vertex_tag.scalar_attribute(vertex_tuples[4]) = input_tag_value_1;
        acc_vertex_tag.scalar_attribute(vertex_tuples[5]) = input_tag_value_1;

        expected_isosurface_vertex_num = 9;
    }
    SECTION("0-4-5")
    {
        const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
        Accessor<long> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
        acc_vertex_tag.scalar_attribute(vertex_tuples[0]) = input_tag_value_1;
        acc_vertex_tag.scalar_attribute(vertex_tuples[4]) = input_tag_value_1;
        acc_vertex_tag.scalar_attribute(vertex_tuples[5]) = input_tag_value_1;

        expected_isosurface_vertex_num = 10;
    }
    SECTION("0-4-5-with-filter")
    {
        const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
        Accessor<long> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
        acc_vertex_tag.scalar_attribute(vertex_tuples[0]) = input_tag_value_1;
        acc_vertex_tag.scalar_attribute(vertex_tuples[4]) = input_tag_value_1;
        acc_vertex_tag.scalar_attribute(vertex_tuples[5]) = input_tag_value_1;

        MeshAttributeHandle<long> filter =
            m.register_attribute<long>("edge_filter", PrimitiveType::Edge, 1);
        const long filter_val = 1;
        filter_tag.emplace_back(std::make_tuple(filter, filter_val));

        Accessor<long> acc_filter = m.create_accessor(filter);
        acc_filter.scalar_attribute(m.edge_tuple_from_vids(0, 1)) = 1;
        acc_filter.scalar_attribute(m.edge_tuple_from_vids(1, 4)) = 1;
        acc_filter.scalar_attribute(m.edge_tuple_from_vids(1, 5)) = 1;
        acc_filter.scalar_attribute(m.edge_tuple_from_vids(2, 5)) = 1;
        acc_filter.scalar_attribute(m.edge_tuple_from_vids(5, 6)) = 1;

        expected_isosurface_vertex_num = 5;
    }

    long expected_vertex_num =
        m.get_all(PrimitiveType::Vertex).size() + expected_isosurface_vertex_num;


    components::internal::Marching mc(m, vertex_tags, output_tags, filter_tag);
    mc.process();

    const auto& vertices = m.get_all(PrimitiveType::Vertex);
    Accessor<long> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
    // vertex number should be correct
    {
        CHECK(vertices.size() == expected_vertex_num);

        long isosurface_vertex_num = 0;
        for (const Tuple& v : vertices) {
            if (acc_vertex_tag.scalar_attribute(v) == isosurface_tag_value) {
                isosurface_vertex_num++;
            }
        }
        CHECK(isosurface_vertex_num == expected_isosurface_vertex_num);
    }

    // should be manifold
    {
        for (const Tuple& v : vertices) {
            if (acc_vertex_tag.scalar_attribute(v) == isosurface_tag_value) {
                std::vector<Tuple> one_ring = simplex::link(m, simplex::Simplex::vertex(v))
                                                  .simplex_vector_tuples(PrimitiveType::Vertex);

                long tagged_neighbors = 0;
                for (const Tuple& neigh : one_ring) {
                    if (acc_vertex_tag.scalar_attribute(neigh) == isosurface_tag_value) {
                        ++tagged_neighbors;
                    }
                }

                if (m.is_boundary_vertex(v)) {
                    CHECK(tagged_neighbors == 1);
                } else {
                    CHECK(tagged_neighbors == 2);
                }
            }
        }
    }

    if (false) {
        wmtk::io::ParaviewWriter
            writer("marching_2d_result", "vertices", m, true, false, true, false);
        m.serialize(writer);
    }
}
