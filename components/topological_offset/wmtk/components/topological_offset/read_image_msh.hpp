#pragma once

#include <map>
#include <string>
#include <wmtk/Types.hpp>

namespace wmtk::components::topological_offset {

// void read_image_msh(
//     const std::string& path,
//     MatrixXd& V_input,
//     MatrixXi& T_input,
//     MatrixXd& T_input_tags,
//     std::map<std::string, int>& tag_label_map);

void read_image_msh(
    const std::string& path,
    MatrixXd& V_input,
    MatrixXi& T_input,
    MatrixXd& T_tag,
    const std::string& tag_label);

} // namespace wmtk::components::topological_offset
