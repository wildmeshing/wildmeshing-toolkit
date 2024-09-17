#pragma once

#include <nlohmann/json.hpp>

#include <wmtk/TriMesh.hpp>


namespace wmtk::components {

std::tuple<std::shared_ptr<TriMesh>, nlohmann::json> tetwild_simplification(
    const TriMesh& mesh,
    const std::string& postion_attr_name,
    double main_esp,
    bool realitive = true,
    double duplicate_tol = -1,
    bool use_sampling = true);


} // namespace wmtk::components