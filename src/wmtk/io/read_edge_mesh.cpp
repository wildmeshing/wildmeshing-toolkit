#include "read_edge_mesh.hpp"

#include <wmtk/utils/Logger.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <igl/remove_duplicate_vertices.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <algorithm>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace wmtk::io {

namespace {

/**
 * @brief Merge vertices closer than `tol` and remap the edges onto the survivors.
 *
 * Degenerate edges (both endpoints merged together) and duplicated edges are dropped.
 * Duplicates are detected on the sorted endpoint pair, so a segment and its reverse
 * count as one, but each surviving edge keeps the ORIENTATION IT WAS GIVEN and the
 * edges keep their input order. Both matter: a curve's orientation is what makes the
 * winding number +/-1 inside it, and normalizing endpoints to (min, max) silently
 * turns every closed loop into an inconsistently oriented one, so the winding-number
 * tagging that decides which faces belong to an input degenerates into noise.
 */
void merge_duplicate_vertices(MatrixXd& V, MatrixXi& E, double tol)
{
    if (tol < 0 || V.rows() == 0) {
        return;
    }

    MatrixXd SV;
    Eigen::VectorXi SVI, SVJ; // SVJ maps old vertex index -> new
    igl::remove_duplicate_vertices(V, tol, SV, SVI, SVJ);

    if (SV.rows() == V.rows() && E.rows() == 0) {
        V = SV;
        return;
    }

    std::set<std::pair<int, int>> seen; // keyed on the sorted pair, for dedup only
    std::vector<std::pair<int, int>> kept; // the edges themselves, as oriented
    kept.reserve(static_cast<size_t>(E.rows()));
    for (int i = 0; i < E.rows(); ++i) {
        const int a = SVJ(E(i, 0));
        const int b = SVJ(E(i, 1));
        if (a == b) {
            continue; // collapsed to a point
        }
        if (seen.insert(std::minmax(a, b)).second) {
            kept.emplace_back(a, b);
        }
    }

    const int n_dropped = static_cast<int>(E.rows()) - static_cast<int>(kept.size());
    if (V.rows() != SV.rows() || n_dropped > 0) {
        logger().info(
            "Merged {} duplicate vertices and dropped {} degenerate/duplicate edges",
            V.rows() - SV.rows(),
            n_dropped);
    }

    V = SV;
    E.resize(kept.size(), 2);
    int idx = 0;
    for (const auto& [a, b] : kept) {
        E(idx, 0) = a;
        E(idx, 1) = b;
        ++idx;
    }
}

} // namespace

void read_edge_mesh(
    const std::string& path,
    MatrixXd& V,
    MatrixXi& E,
    double tol_rel,
    double tol_abs)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        log_and_throw_error("Could not open file: {}", path);
    }

    std::vector<Vector3d> v_list;
    std::vector<Vector2i> e_list;

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        std::istringstream iss(line);
        std::string type;
        iss >> type;

        if (type == "v") {
            double x = 0, y = 0, z = 0;
            iss >> x >> y;
            if (iss) { // If it could read 2
                double temp;
                if (iss >> temp) {
                    z = temp;
                }
            }
            v_list.emplace_back(x, y, z);
        } else if (type == "l") {
            std::string token;
            std::vector<int> vs;
            while (iss >> token) {
                size_t slash_pos = token.find('/');
                std::string v_idx_str = token.substr(0, slash_pos);
                if (v_idx_str.empty()) {
                    continue;
                }

                int v_idx = std::stoi(v_idx_str);
                if (v_idx < 0) {
                    v_idx = v_list.size() + v_idx + 1;
                }
                vs.push_back(v_idx - 1);
            }
            for (size_t i = 0; i + 1 < vs.size(); i++) {
                e_list.emplace_back(vs[i], vs[i + 1]);
            }
        }
    }

    V.resize(v_list.size(), 3);
    for (size_t i = 0; i < v_list.size(); i++) {
        V.row(i) = v_list[i];
    }

    E.resize(e_list.size(), 2);
    for (size_t i = 0; i < e_list.size(); i++) {
        E.row(i) = e_list[i];
    }

    double tol = tol_abs;
    if (tol < 0 && tol_rel >= 0 && V.rows() > 0) {
        const double diag = (V.colwise().maxCoeff() - V.colwise().minCoeff()).norm();
        tol = tol_rel * diag;
    }
    merge_duplicate_vertices(V, E, tol);
}

} // namespace wmtk::io