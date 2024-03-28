#include <fstream>
#include <wmtk/components/adaptive_tessellation/function/utils/AnalyticalFunctionAvgDistanceToLimit.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/TextureMapAvgDistanceToLimit.hpp>
#include "ATData.hpp"
#include "ATOperations.hpp"


namespace wmtk::components::operations::internal {
using namespace wmtk;
using namespace wmtk::attribute;
void _debug_sampling(
    std::shared_ptr<Mesh> uv_mesh_ptr,
    wmtk::attribute::MeshAttributeHandle m_uv_handle,
    wmtk::components::function::utils::ThreeChannelPositionMapEvaluator& image_sampling,
    wmtk::components::function::utils::ThreeChannelPositionMapEvaluator& func_eval)
{
    nlohmann::ordered_json jsonData_image;
    nlohmann::ordered_json jsonData_func;

    std::cout << "debug sampling" << std::endl;
    // get the accessors
    auto m_uv_accessor = uv_mesh_ptr->create_accessor(m_uv_handle.as<double>());
    const auto vertices = uv_mesh_ptr->get_all(PrimitiveType::Vertex);
    for (const auto& v : vertices) {
        Eigen::Vector2d uv = m_uv_accessor.vector_attribute(v);

        auto image_xyz = image_sampling.uv_to_position<double>(uv);
        auto func_xyz = func_eval.uv_to_position<double>(uv);
        jsonData_image.push_back({{"x", image_xyz(0)}, {"y", image_xyz(1)}, {"z", image_xyz(2)}});
        jsonData_func.push_back({{"x", func_xyz(0)}, {"y", func_xyz(1)}, {"z", func_xyz(2)}});
    }
    // Open the file in append mode
    std::ofstream outputFileimage("ATData_debug_sampling_image.json");
    outputFileimage << jsonData_image.dump(4);
    outputFileimage.close();
    std::cout << "JSON data written to "
              << "ATData_debug_sampling.json" << std::endl;
    std::ofstream outputFilefunc("ATData_debug_sampling_func.json");
    outputFilefunc << jsonData_func.dump(4);
    outputFilefunc.close();
    std::cout << "JSON data written to "
              << "ATData_debug_sampling.json" << std::endl;
}


void _debug_texture_integral(
    std::shared_ptr<Mesh> mesh,
    wmtk::attribute::MeshAttributeHandle m_uv_handle,
    wmtk::components::function::utils::ThreeChannelPositionMapEvaluator& image_evaluator,
    wmtk::components::function::utils::ThreeChannelPositionMapEvaluator& func_evaluator)
{
    Accessor<double> m_uv_accessor = mesh->create_accessor(m_uv_handle.as<double>());
    wmtk::attribute::MeshAttributeHandle image_res_handle =
        mesh->register_attribute<double>("image_res", PrimitiveType::Triangle, 1);
    Accessor<double> image_res_accessor = mesh->create_accessor(image_res_handle.as<double>());
    wmtk::attribute::MeshAttributeHandle func_res_handle =
        mesh->register_attribute<double>("func_res", PrimitiveType::Triangle, 1);
    Accessor<double> func_res_accessor = mesh->create_accessor(func_res_handle.as<double>());
    for (auto& f : mesh->get_all(PrimitiveType::Triangle)) {
        if (!mesh->is_ccw(f)) {
            f = mesh->switch_tuple(f, PrimitiveType::Vertex);
        }
        const Eigen::Vector2d uv0 = m_uv_accessor.vector_attribute(f);
        const Eigen::Vector2d uv1 =
            m_uv_accessor.vector_attribute(mesh->switch_tuple(f, PrimitiveType::Vertex));
        const Eigen::Vector2d uv2 = m_uv_accessor.vector_attribute(
            mesh->switch_tuple(mesh->switch_tuple(f, PrimitiveType::Edge), PrimitiveType::Vertex));

        wmtk::components::function::utils::AnalyticalFunctionAvgDistanceToLimit
            analytical_quadrature(func_evaluator);
        double func_res = analytical_quadrature.distance(uv0, uv1, uv2);
        func_res_accessor.scalar_attribute(f) = func_res;

        wmtk::components::function::utils::TextureMapAvgDistanceToLimit texture_integral(
            image_evaluator);
        double image_res = texture_integral.distance(uv0, uv1, uv2);
        image_res_accessor.scalar_attribute(f) = image_res;
        std::cout << "func_res: " << func_res << std::endl;
        std::cout << "image_res: " << image_res << std::endl;
    }
    // write(mesh, "diff_sampling_debug_og_l4", "at_sampling_flat__xyz_output", 0, 1);
}

} // namespace wmtk::components::operations::internal