#pragma once
#include <wmtk/Types.hpp>


namespace wmtk::components::topological_offset {


/**
 * @brief read vertex and tri/tet data from msh (based on whether any tets are in msh).
 * Tags are assumed to be labeled sequentially from tag_0
 */
void read_image_msh(
    const std::string& path,
    MatrixXd& V_input,
    MatrixXi& F_input,
    MatrixXd& F_input_tags);


}