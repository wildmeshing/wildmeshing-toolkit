#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <tools/DEBUG_TetMesh.hpp>
#include <tools/DEBUG_TriMesh.hpp>
#include <tools/TetMesh_examples.hpp>
#include <tools/TriMesh_examples.hpp>
#include <wmtk/components/multimesh_from_tag/internal/MultiMeshFromTag.hpp>
#include <wmtk/components/multimesh_from_tag/internal/MultiMeshFromTagOptions.hpp>
#include <wmtk/components/multimesh_from_tag/multimesh_from_tag.hpp>

using json = nlohmann::json;
using namespace wmtk;
using namespace tests;
using namespace components;
using namespace internal;

TEST_CASE("multimesh_from_tag_tri_tri", "[components][multimesh][multimesh_from_tag][.]")
{
    auto mesh_in = tests::disk(6);
    DEBUG_TriMesh& m = static_cast<DEBUG_TriMesh&>(*mesh_in);

    auto tag_handle = m.register_attribute<int64_t>("tag", PrimitiveType::Triangle, 1);
    int64_t tag_value = 1;

    auto tag_acc = m.create_accessor<int64_t>(tag_handle);
    tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 1, 2)) = 1;
    tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 2, 3)) = 1;
    tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 3, 4)) = 1;
    tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 5, 6)) = 1;

    MultiMeshFromTag mmft(m, tag_handle, tag_value);
    mmft.compute_substructure_ids();
}