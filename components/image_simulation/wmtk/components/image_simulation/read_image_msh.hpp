#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <wmtk/Types.hpp>

namespace wmtk::components::image_simulation {

struct InputData
{
    MatrixXd V_input;
    MatrixXr V_input_r;
    MatrixXi T_input;
    MatrixXi T_input_tag;

    MatrixXd V_envelope;
    MatrixXi F_envelope;
};

/**
 * @brief Read a .msh file that contains tags.
 */
InputData read_image_msh(const std::string& path);

/**
 * @brief Read one or multiple images and convert them into a tet mesh.
 *
 * The color in the image become tet tags.
 */
InputData read_image(
    const std::vector<std::string>& input_paths,
    const std::string& output_filename,
    const nlohmann::json& json_params);

} // namespace wmtk::components::image_simulation