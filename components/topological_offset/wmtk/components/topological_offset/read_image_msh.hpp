#pragma once
#include <wmtk/Types.hpp>


namespace wmtk::components::topological_offset {


void read_image_msh(
    const std::string& path,
    MatrixXd& V_input,
    MatrixXi& F_input,
    MatrixXd& F_input_tags,
    const std::string& tag_name,
    std::vector<std::string>& all_tag_names);


}