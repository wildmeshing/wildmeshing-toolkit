#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
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

TEST_CASE("marching_component", "[components][marching][.]")
{
    const long input_tag_value_0 = 0;
    const long input_tag_value_1 = 1;
    const long isosurface_tag_value = 2;

    tests::DEBUG_TriMesh m = wmtk::tests::hex_plus_two_with_position();

    MeshAttributeHandle<long> vertex_tag_handle = m.register_attribute<long>(
        "vertex_tag",
        PrimitiveType::Vertex,
        1,
        false,
        input_tag_value_0);
    MeshAttributeHandle<long> edge_tag_handle =
        m.register_attribute<long>("edge_tag", PrimitiveType::Edge, 1, false, input_tag_value_0);

    std::tuple<MeshAttributeHandle<long>, long, long> vertex_tags =
        std::make_tuple(vertex_tag_handle, input_tag_value_0, input_tag_value_1);

    std::vector<std::tuple<MeshAttributeHandle<long>, long>> output_tags;
    output_tags.emplace_back(std::make_tuple(vertex_tag_handle, isosurface_tag_value));
    output_tags.emplace_back(std::make_tuple(edge_tag_handle, isosurface_tag_value));

    std::vector<std::tuple<MeshAttributeHandle<long>, long>> filter_tag;

    SECTION("2d_case -- should be manifold")
    {
        //    0---1---2
        //   / \ / \ / \ .
        //  3---4---5---6
        //   \ / \ /  .
        //    7---8
        // set edge 4 as input
        {
            const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
            Accessor<long> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
            acc_vertex_tag.scalar_attribute(vertex_tuples[4]) = input_tag_value_1;
        }

        components::internal::Marching mc(m, vertex_tags, output_tags, filter_tag);
        mc.process();

        // offset edge number should be correct
        {
            long offset_num = 0;
            Accessor<long> acc_edge_tag = m.create_accessor<long>(edge_tag_handle);
            for (const Tuple& t : m.get_all(wmtk::PrimitiveType::Edge)) {
                if (acc_edge_tag.scalar_attribute(t) == isosurface_tag_value) {
                    offset_num++;
                }
            }
            CHECK(offset_num == 6);
        }

        // should be manifold
        {
            Accessor<long> acc_edge_tag = m.create_accessor(edge_tag_handle);
            for (const Tuple& edge : m.get_all(PrimitiveType::Edge)) {
                if (acc_edge_tag.scalar_attribute(edge) == isosurface_tag_value) {
                    Tuple t = m.switch_face(m.switch_edge(edge));
                    int neighbor_num = 0;
                    while (t != edge) {
                        if (acc_edge_tag.scalar_attribute(t) == isosurface_tag_value) {
                            ++neighbor_num;
                        }
                        t = m.switch_face(m.switch_edge(t));
                    }
                    CHECK(neighbor_num == 1);
                }
            }
        }

        if (false) {
            wmtk::io::ParaviewWriter
                writer(data_dir / "marching_2d_result", "vertices", m, true, true, true, false);
            m.serialize(writer);
        }
    }
}
