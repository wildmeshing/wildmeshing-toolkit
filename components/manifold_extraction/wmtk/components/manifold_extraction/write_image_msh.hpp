#include <Eigen/Core>
#include <map>
#include <wmtk/utils/io.hpp>

using namespace Eigen;

void write_image_msh(
    std::string opath,
    const MatrixXd& V,
    const MatrixXi& T,
    const MatrixXd& Tags,
    const std::map<std::string, int>& label_map);
