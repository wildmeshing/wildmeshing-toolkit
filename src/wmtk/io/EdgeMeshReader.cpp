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
    char* line_buffer = nullptr;
    std::fstream f(m_filename);
    if (!f) {
        throw std::runtime_error("can't open file!");
    }
    if (m_type == OBJ) {
        std::string buffer;
        while (!f.eof()) {
            getline(f, buffer);
            // here is for debugging
            // spdlog::info("lineinfo: {}", buffer[0]);
            if (buffer[0] != 'v' && buffer[0] != 'l') continue;
            // read obj
            line_buffer = new char[buffer.size()];
            strcpy(line_buffer, buffer.c_str());
            if (line_buffer[0] == 'v') {
                std::vector<double> positions;
                char* token = strtok(line_buffer, " ");
                while (token != NULL) {
                    if (isdigit(token[0]) || token[0] == '-' || token[0] == '.') {
                        positions.push_back(std::atof(token));
                    }
                    token = strtok(NULL, " ");
                }
                vertices.push_back(positions);
            } else if (line_buffer[0] == 'l') {
                line_buffer = new char[buffer.size()];
                strcpy(line_buffer, buffer.c_str());
                std::vector<long> segment;
                char* token = strtok(line_buffer, " ");
                while (token != NULL) {
                    if (isdigit(token[0])) {
                        segment.push_back(std::atol(token));
                    }
                    token = strtok(NULL, " ");
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
            if (line_buffer) delete[] line_buffer;
        }
    } else if (m_type == OFF) {
        throw std::runtime_error(".off has not implemented!");
        abort();
        std::string buffer;
        while (getline(f, buffer)) {
            // read off
        }
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
} // namespace wmtk