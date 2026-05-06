#pragma once

#include <map>
#include <string>
#include <wmtk/Types.hpp>

namespace wmtk::components::manifold_extraction {

struct InputData
{
    MatrixXd V_input;
    MatrixXi T_input;
    MatrixSi T_input_tags;
    std::vector<std::string> tag_names;

    // note: not actually used, just for spitting back to output
    MatrixXd V_envelope;
    MatrixXi F_envelope;
};

/**
 * @brief read vertex and cell data from msh. physical groups are detected if present.
 * @note if "EnvelopeSurface" is present it will be extracted and retained
 */
InputData read_image_msh(const std::string& path);

} // namespace wmtk::components::manifold_extraction
