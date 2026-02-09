#include "write_image_msh.hpp"

using namespace Eigen;

void write_image_msh(
    std::string opath,
    const MatrixXd& V,
    const MatrixXi& T,
    const MatrixXd& Tags,
    const std::map<std::string, int>& label_map)
{
    wmtk::MshData msh;

    msh.add_tet_vertices(V.rows(), [&](size_t i) { return V.row(i); });

    msh.add_tets(T.rows(), [&](size_t i) { return T.row(i); });

    for (const auto& pair : label_map) {
        int ind = pair.second;
        msh.add_tet_attribute<1>(pair.first, [&](size_t i) { return Tags(i, ind); });
    }

    msh.save(opath, true);
}