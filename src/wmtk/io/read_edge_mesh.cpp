#include "read_edge_mesh.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace wmtk::io {

void read_edge_mesh(const std::string& path, MatrixXd& V, MatrixXi& E)
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
}

} // namespace wmtk::io