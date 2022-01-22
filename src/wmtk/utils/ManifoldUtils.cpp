#include "ManifoldUtils.hpp"

#include <Eigen/src/Core/Array.h>
#include <igl/extract_manifold_patches.h>
#include <igl/remove_unreferenced.h>

bool wmtk::separate_to_manifold(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::array<size_t, 3>>& faces,
    std::vector<Eigen::Vector3d>& out_v,
    std::vector<std::array<size_t, 3>>& out_f)
{
    Eigen::MatrixXi F(faces.size(), 3);
    for (auto i = 0; i < faces.size(); i++) F.row(i) << faces[i][0], faces[i][1], faces[i][2];
    Eigen::MatrixXi V(vertices.size(), 3);
    for (auto i = 0; i < vertices.size(); i++) V.row(i) = vertices[i];

    Eigen::VectorXi indices;
    auto num_patch = igl::extract_manifold_patches(F, indices);
    assert(num_patch == indices.maxCoeff() + 1);
    if (num_patch == 1) {
        out_v = vertices;
        out_f = faces;
        return false;
    }

    std::vector<std::vector<int>> patches(num_patch);
    for (auto i = 0; i < indices.size(); i++) {
        patches[indices[i]].push_back(i);
    } // this also ensure it is sorted.

    out_v.clear();
    out_v.reserve(vertices.size() * 2);
    out_f.clear();
    out_f.reserve(faces.size());
    auto current_vnum = 0;
    for (auto i = 0; i < patches.size(); i++) {
        auto& patch = patches[i];
        Eigen::MatrixXi pF(patch.size(), 3);
        for (auto j = 0; j < pF.rows(); j++) {
            pF.row(j) = F.row(patch[j]);
        }
        Eigen::MatrixXd NV;
        Eigen::ArrayXi NF;
        Eigen::VectorXi I;
        igl::remove_unreferenced(V, pF, NV, NF, I);
        NF += current_vnum;
        current_vnum += NV.rows();
        // assign
        for (auto j = 0; j < NV.rows(); j++) out_v.push_back(NV.row(j));
        for (auto j = 0; j < NF.rows(); j++)
            out_f.push_back(
                std::array<size_t, 3>{{size_t(NF(j, 0)), size_t(NF(j, 1)), size_t(NF(j, 2))}});
    }
    return true;
}