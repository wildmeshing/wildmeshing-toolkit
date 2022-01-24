#include "ManifoldUtils.hpp"

#include <Eigen/src/Core/Array.h>
#include <igl/extract_manifold_patches.h>
#include <igl/remove_unreferenced.h>
#include "spdlog/spdlog.h"
#include <igl/is_edge_manifold.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/fmt/bundled/ranges.h>

bool wmtk::separate_to_manifold(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::array<size_t, 3>>& faces,
    std::vector<Eigen::Vector3d>& out_v,
    std::vector<std::array<size_t, 3>>& out_f)
{
    Eigen::MatrixXi F(faces.size(), 3);
    for (auto i = 0; i < faces.size(); i++) F.row(i) << faces[i][0], faces[i][1], faces[i][2];
    Eigen::MatrixXd V(vertices.size(), 3);
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
        spdlog::critical("patch {}", patch);
        Eigen::MatrixXi pF(patch.size(), 3);
        for (auto j = 0; j < pF.rows(); j++) {
            pF.row(j) = F.row(patch[j]);
        }
        Eigen::MatrixXd NV;
        Eigen::ArrayXXi NF;
        Eigen::VectorXi I;
        igl::remove_unreferenced(V, pF, NV, NF, I);
        NF += current_vnum;
        current_vnum += NV.rows();
        // assign
        for (auto j = 0; j < NV.rows(); j++) out_v.push_back(NV.row(j));
        for (auto j = 0; j < NF.rows(); j++)
            out_f.push_back(
                std::array<size_t, 3>{{size_t(NF(j, 0)), size_t(NF(j, 1)), size_t(NF(j, 2))}});

        {
            Eigen::MatrixXd outV;
            Eigen::MatrixXi outF;
            outV.resize(out_v.size(), 3);
            outF.resize(out_f.size(), 3);
            for (auto i = 0; i < out_v.size(); i++) {
                outV.row(i) = out_v[i];
            }
            for (auto i = 0; i < out_f.size(); i++) {
                outF.row(i) << out_f[i][0], out_f[i][1], out_f[i][2];
            }
            if(!igl::is_edge_manifold(outF)) spdlog::critical("i {}", i);
        }
        exit(0);
    }
    return true;
}