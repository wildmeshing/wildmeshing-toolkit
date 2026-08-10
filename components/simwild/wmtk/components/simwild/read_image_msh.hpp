#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <wmtk/Types.hpp>

namespace wmtk::components::simwild {

struct InputData
{
    MatrixXd V_input;
    MatrixXr V_input_r;
    MatrixXi T_input;
    MatrixSi T_input_tag;
    std::vector<std::string> tag_names;

    MatrixXd V_envelope;
    MatrixXi F_envelope;
};

/**
 * @brief Read a .msh file that contains tags.
 */
InputData read_image_msh(const std::string& path);

/**
 * @brief Read one or multiple meshes and convert them into a tet mesh.
 *
 * Winding number is used to tag inside/outside.
 */
InputData read_mesh(
    const std::vector<std::string>& input_paths,
    const std::string& output_filename,
    const nlohmann::json& json_params);

/**
 * @brief Read one or multiple 2D curve networks and convert them into a tagged triangle mesh.
 *
 * The 2D counterpart of read_mesh: exact arrangement of the curves, then a per-input winding
 * number to tag which faces belong to which material. See EmbedCurves.
 */
InputData read_curves(
    const std::vector<std::string>& input_paths,
    const std::string& output_filename,
    const nlohmann::json& json_params);

/**
 * @brief Decide whether an input is 2D or 3D.
 *
 * `dimension` is "auto" (the default), 2 or 3. Auto looks at the first input:
 *   - .msh                            -> 0, meaning "the msh reader decides from the content"
 *   - OBJ/OFF/STL with any 'f' record -> 3, a surface for EmbedSurface
 *   - OBJ with 'l' but no 'f'         -> 2, a curve network for EmbedCurves
 *
 * An explicit 2 or 3 forces the route; it does not skip the sniff, it disagrees with it
 * loudly, because the failure otherwise surfaces as a confusing parse error much later.
 */
int resolve_input_dimension(
    const std::vector<std::string>& input_paths,
    const nlohmann::json& json_params);

} // namespace wmtk::components::simwild