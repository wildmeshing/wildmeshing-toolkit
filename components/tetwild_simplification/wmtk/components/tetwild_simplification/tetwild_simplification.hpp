#pragma once

#include <nlohmann/json.hpp>

#include <wmtk/TriMesh.hpp>


namespace wmtk::components {

/**
 * @brief Perform the simplification from the original tetwild.
 *
 * This code does not use WMTK for anything besides in the API.
 *
 * @param mesh The mesh that should be simplified.
 * @param postion_attr_name The name of the position attribute.
 * @param main_eps Specifies the amount of simplification.
 * @param relative The `main_eps` and `duplicate_tol` are relative to the bbox diagonal.
 * @param duplicate_tol The radius in which vertices are considered duplicates. Also, faces that
 * have an area below this value are removed.
 * @param use_sampling Use sampling for the envelope.
 */
std::tuple<std::shared_ptr<TriMesh>, nlohmann::json> tetwild_simplification(
    const TriMesh& mesh,
    const std::string& postion_attr_name,
    double main_eps,
    bool relative = true,
    double duplicate_tol = -1,
    bool use_sampling = true);


} // namespace wmtk::components