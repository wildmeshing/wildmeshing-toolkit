#pragma once

#include <string>
#include <map>
#include <wmtk/Types.hpp>

namespace wmtk::components::topological_offset {

void read_image_msh(
    const std::string& path,
    MatrixXd& V_input,
    MatrixXi& T_input,
    MatrixXd& T_input_tags,
    std::map<std::string, int>& tag_label_map);
}
