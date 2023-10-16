#include "EdgeMeshReader.hpp"
#include <spdlog/spdlog.h>
#include <map>

namespace wmtk {
EdgeMeshReader::EdgeMeshReader(const std::string& filename, const file_type type)
    : m_filename(filename)
    , m_type(type)
{}
void EdgeMeshReader::read(
    Eigen::Matrix<long, -1, -1>& edges,
    Eigen::MatrixXd& vertices,
    Eigen::MatrixXd& vertices_w,
    Eigen::MatrixXd& vertices_texture,
    Eigen::MatrixXd& vertices_normal,
    Eigen::MatrixXd& vertices_parameter)
{
    std::vector<std::pair<long, long>> vec_edges;
    std::vector<std::vector<double>> vec_vertices;
    std::vector<double> vec_vertices_w;
    std::vector<std::vector<double>> vec_vertices_texture;
    std::vector<std::vector<double>> vec_vertices_normal;
    std::vector<std::vector<double>> vec_vertices_parameter;
    switch (m_type) {
    case OBJ:
        read_obj(
            vec_edges,
            vec_vertices,
            vec_vertices_w,
            vec_vertices_texture,
            vec_vertices_normal,
            vec_vertices_parameter);
        break;
    case OFF: read_off(vec_edges, vec_vertices); break;
    default: throw std::runtime_error("Unknown data type!"); break;
    }

    edges.resize(vec_edges.size(), 2);
    vertices.resize(vec_vertices.size(), 3);
    vertices_w.resize(vec_vertices.size(), 1);
    vertices_texture.resize(vec_vertices_texture.size(), 3);
    vertices_normal.resize(vec_vertices_normal.size(), 3);
    vertices_parameter.resize(vec_vertices_parameter.size(), 3);

    for (size_t i = 0; i < edges.rows(); i++) {
        edges(i, 0) = vec_edges[i].first;
        edges(i, 1) = vec_edges[i].second;
    }
    for (size_t i = 0; i < vertices.rows(); i++) {
        vertices(i, 0) = vec_vertices[i][0];
        vertices(i, 1) = vec_vertices[i][1];
        vertices(i, 2) = vec_vertices[i][2];
        vertices_w(i, 0) = vec_vertices[i][3];
    }
    for (size_t i = 0; i < vertices_texture.rows(); i++) {
        vertices_texture(i, 0) = vec_vertices_texture[i][0];
        vertices_texture(i, 1) = vec_vertices_texture[i][1];
        vertices_texture(i, 2) = vec_vertices_texture[i][2];
    }
    for (size_t i = 0; i < vertices_normal.rows(); i++) {
        vertices_normal(i, 0) = vec_vertices_normal[i][0];
        vertices_normal(i, 1) = vec_vertices_normal[i][1];
        vertices_normal(i, 2) = vec_vertices_normal[i][2];
    }
    for (size_t i = 0; i < vertices_parameter.rows(); i++) {
        vertices_parameter(i, 0) = vec_vertices_parameter[i][0];
        vertices_parameter(i, 1) = vec_vertices_parameter[i][1];
        vertices_parameter(i, 2) = vec_vertices_parameter[i][2];
    }
}

void EdgeMeshReader::read_obj(
    std::vector<std::pair<long, long>>& edges,
    std::vector<std::vector<double>>& vertices,
    std::vector<double>& vertices_w,
    std::vector<std::vector<double>>& vertices_texture,
    std::vector<std::vector<double>> vertices_normal,
    std::vector<std::vector<double>>& vertices_parameter)
{
    std::ifstream f(m_filename);
    if (!f.is_open()) {
        throw std::runtime_error("can't open file!");
    }

    std::string buffer;
    while (!f.eof()) {
        getline(f, buffer);
        std::vector<std::string> tokens;
        std::stringstream line(buffer);
        std::string token;
        getline(line, token, ' ');
        switch (str_to_type(token)) {
        case V: {
            std::vector<double> positions;
            while (getline(line, token, ' ')) {
                // spdlog::info("lineinfo: {}", token);
                positions.push_back(std::atof(token.c_str()));
            }
            assert(positions.size() >= 3);
            if (positions.size() == 4) {
                vertices_w.push_back(positions[3]);
            }
            vertices.push_back(positions);
        }; break;
        case L: {
            std::vector<long> segment;
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
        }; break;
        case VT: {
            std::vector<double> texture;
            while (getline(line, token, ' ')) {
                if (token[0] == '[') {
                    token.erase(std::remove(token.begin(), token.end(), ']'), token.end());
                    texture.push_back(std::atof(token.c_str() + 1));
                } else {
                    texture.push_back(std::atof(token.c_str()));
                }
            }
            assert(texture.size() >= 2);
            if (texture.size() == 2) {
                texture.push_back(0);
            }
            vertices_texture.push_back(texture);
        }; break;
        case VN: {
            std::vector<double> normal;
            while (getline(line, token, ' ')) {
                normal.push_back(std::atof(token.c_str()));
            }
            assert(normal.size() == 3);
            double standard =
                sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
            normal[0] /= standard;
            normal[1] /= standard;
            normal[2] /= standard;
            vertices_normal.push_back(normal);
        }; break;
        case VP: {
            std::vector<double> parameters;
            while (getline(line, token, ' ')) {
                parameters.push_back(std::atof(token.c_str()));
            }
            assert(parameters.size() == 3);
            vertices_parameter.push_back(parameters);
        }; break;
        default: continue;
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