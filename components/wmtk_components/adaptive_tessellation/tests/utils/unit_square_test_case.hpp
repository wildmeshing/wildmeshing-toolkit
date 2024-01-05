#include <set>
#include <tools/DEBUG_TriMesh.hpp>
#include <tools/TriMesh_examples.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/TextureIntegral.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/ThreeChannelPositionMapEvaluator.hpp>
#include <wmtk/components/adaptive_tessellation/operations/internal/ATData.hpp>
#include <wmtk/multimesh/utils/create_tag.hpp>
#include <wmtk/multimesh/utils/edge_meshes_parameterization.hpp>
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>
#include <wmtk/multimesh/utils/map_sibling_edge_meshes.hpp>
namespace AT = wmtk::components::adaptive_tessellation;
namespace ATfunction = wmtk::components::adaptive_tessellation::function;
using namespace wmtk;
using namespace wmtk::tests;
std::pair<std::vector<std::shared_ptr<Mesh>>, std::map<Mesh*, Mesh*>> unit_square_example(
    DEBUG_TriMesh& uv_mesh,
    DEBUG_TriMesh& position_mesh,
    std::array<AT::image::Image, 3>& images)
{
    ATfunction::utils::ThreeChannelPositionMapEvaluator evaluator(images);

    // get uv coordinate accessor
    wmtk::attribute::MeshAttributeHandle uv_handle =
        uv_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    wmtk::attribute::ConstAccessor<double> uv_acc =
        uv_mesh.create_const_accessor(uv_handle.as<double>());

    // create the position mesh attribute vertices and set it to the mapped uv coordinates
    wmtk::attribute::MeshAttributeHandle position_handle =
        position_mesh.register_attribute<double>("vertices", PrimitiveType::Vertex, 3);
    Accessor<double> pos_acc = position_mesh.create_accessor(position_handle.as<double>());

    for (auto& v : uv_mesh.get_all(PrimitiveType::Vertex)) {
        Eigen::Vector2<double> uv = uv_acc.const_vector_attribute(v);

        Eigen::Vector3<double> pos = evaluator.uv_to_position(uv);
        auto v1 = uv_mesh.id(v, PrimitiveType::Vertex);
        auto v2 = uv_mesh.id(uv_mesh.switch_vertex(v), PrimitiveType::Vertex);
        auto fid = uv_mesh.id(v, PrimitiveType::Face);
        Tuple pos_tuple = position_mesh.edge_tuple_between_v1_v2(v1, v2, fid);
        pos_acc.vector_attribute(pos_tuple) = pos;
    }

    std::set<int64_t> critical_vids = {0, 1, 2, 3};
    auto tags = wmtk::multimesh::utils::create_tags(uv_mesh, critical_vids);

    std::vector<std::shared_ptr<Mesh>> edge_mesh_ptrs;
    for (int64_t tag : tags) {
        edge_mesh_ptrs.emplace_back(
            wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                uv_mesh,
                "edge_tag",
                tag,
                PrimitiveType::Edge));
    }
    std::map<Mesh*, Mesh*> sibling_meshes_map =
        AT::multimesh::utils::map_sibling_edge_meshes(edge_mesh_ptrs);

    AT::multimesh::utils::parameterize_all_edge_meshes(uv_mesh, edge_mesh_ptrs, sibling_meshes_map);
    return {edge_mesh_ptrs, sibling_meshes_map};
}