#pragma once

#include <string>
#include <wmtk/Types.hpp>

namespace wmtk::components::tet_implicits {

void read_image_msh(
    const std::string& path,
    MatrixXd& V_input,
    MatrixXi& T_input,
    MatrixXi& T_input_tag,
    MatrixXd& V_envelope,
    MatrixXi& F_envelope);

}