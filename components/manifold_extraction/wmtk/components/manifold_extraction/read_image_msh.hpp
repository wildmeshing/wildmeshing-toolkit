#pragma once

#include <map>
#include <string>
#include <wmtk/Types.hpp>

namespace wmtk::components::manifold_extraction {


void read_image_msh(
    const std::string& path,
    MatrixXd& V_input,
    MatrixXi& T_input,
    MatrixXd& T_tag,
    const std::string& tag_label);

} // namespace wmtk::components::manifold_extraction
