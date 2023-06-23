#pragma once

#include <igl/readOFF.h>
#include <map>
#include <nlohmann/json.hpp>
#include <wmtk/utils/Logger.hpp>
#include "internal/MeshInfoOptions.h"

namespace wmtk {
namespace components {
inline void mesh_info(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;

    MeshInfoOptions options = j.get<MeshInfoOptions>();

    std::filesystem::path& file = files[options.input];

    // pseudo load
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::readOFF(file.string(), V, F);

    wmtk::logger().info("Mesh: {}", file.string());
    wmtk::logger().info("Vertices: {}", V.rows());
    wmtk::logger().info("Faces: {}", F.rows());
}
} // namespace components
} // namespace wmtk