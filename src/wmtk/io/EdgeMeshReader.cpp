#include "EdgeMeshReader.hpp"
#include <spdlog/spdlog.h>

namespace wmtk {
EdgeMeshReader::EdgeMeshReader(const std::string& filename, const file_type type)
    : m_filename(filename)
    , m_type(type)
{}
void EdgeMeshReader::read(Eigen::Matrix<long, -1, -1>& E, Eigen::MatrixXd& V)
{
    std::vector<std::pair<long, long>> edges;
    std::vector<std::vector<double>> vertices;

    switch (m_type) {
    case OBJ: read_obj(edges, vertices); break;
    case OFF: read_off(edges, vertices); break;
    default: throw std::runtime_error("Unknown data type!"); break;
    }

    E.resize(edges.size(), 2);
    V.resize(vertices.size(), 3);

    for (size_t i = 0; i < E.rows(); i++) {
        E(i, 0) = edges[i].first;
        E(i, 1) = edges[i].second;
    }
    for (size_t i = 0; i < V.rows(); i++) {
        V(i, 0) = vertices[i][0];
        V(i, 1) = vertices[i][1];
        V(i, 2) = vertices[i][2];
    }
}

void EdgeMeshReader::read_obj(
    std::vector<std::pair<long, long>>& edges,
    std::vector<std::vector<double>>& vertices)
{
    std::ifstream f(m_filename);
    if (!f.is_open()) {
        throw std::runtime_error("can't open file!");
    }
    std::string buffer;
    while (!f.eof()) {
        getline(f, buffer);
        std::vector<std::string> tokens;
        // here is for debugging
        // spdlog::info("lineinfo: {}", buffer[0]);
        // read obj
        if (buffer[0] == 'v') {
            std::vector<double> positions;
            std::stringstream line(buffer);
            std::string token;
            getline(line, token, ' ');
            while (getline(line, token, ' ')) {
                // spdlog::info("lineinfo: {}", token);
                positions.push_back(std::atof(token.c_str()));
            }
            vertices.push_back(positions);
        } else if (buffer[0] == 'l') {
            std::vector<long> segment;
            std::stringstream line(buffer);
            std::string token;
            getline(line, token, ' ');
            while (getline(line, token, ' ')) {
                segment.push_back(std::atol(token.c_str()));
            }
            if (segment.size() > 1) {
                long v0, v1;
                v0 = segment[0];
                for (size_t i = 1; i < segment.size(); ++i) {
                    v1 = segment[i];
                    std::pair<long, long> edge;
                    edge.first = v0;
                    edge.second = v1;
                    edges.push_back(edge);
                    v0 = v1;
                }
            }
        }
    }
}
void EdgeMeshReader::read_off(
    std::vector<std::pair<long, long>>& edges,
    std::vector<std::vector<double>>& vertices)
{
    throw std::runtime_error("OFF is not supported!");
}
} // namespace wmtk