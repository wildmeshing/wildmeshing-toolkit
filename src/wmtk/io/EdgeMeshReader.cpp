#include "EdgeMeshReader.hpp"
#include <spdlog/spdlog.h>
#include <map>

namespace wmtk {
EdgeMeshReader::EdgeMeshReader(const std::filesystem::path& filename, const file_type type)
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
    case OFF:;
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
    std::vector<std::vector<double>>& vertices_normal,
    std::vector<std::vector<double>>& vertices_parameter)
{
    std::ifstream f(m_filename);
    if (!f.is_open()) {
        throw std::runtime_error("can't open file!");
    }

    std::string buffer;
    while (getline(f, buffer)) {
        buffer.erase(std::remove(buffer.begin(), buffer.end(), '['), buffer.end());
        buffer.erase(std::remove(buffer.begin(), buffer.end(), ']'), buffer.end());
        std::istringstream iss(buffer);
        std::string token;
        iss >> token;
        switch (str_to_type(token)) {
        case V: {
            std::vector<double> positions;
            double x, y, z, w;
            if (iss >> x >> y >> z) {
                positions.push_back(x);
                positions.push_back(y);
                positions.push_back(z);
                if (iss >> w) {
                    vertices_w.push_back(w);
                } else {
                    vertices_w.push_back(0);
                }
            } else {
                throw std::runtime_error("file format error!");
            }
            vertices.push_back(positions);
        }; break;
        case L: {
            std::vector<long> segment;
            long idx_data;
            while (iss >> idx_data) {
                segment.push_back(idx_data);
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
            } else {
                spdlog::info("Warning, this file contains idividual points! Points ignored...");
            }
        }; break;
        case VT: {
            std::vector<double> texture;
            double u, v, w;
            if (iss >> u >> v) {
                texture.push_back(u);
                texture.push_back(v);
                if (iss >> w) {
                    texture.push_back(w);
                } else {
                    texture.push_back(0);
                }
            } else {
                throw std::runtime_error("file format error!");
            }
            vertices_texture.push_back(texture);
        }; break;
        case VN: {
            std::vector<double> normal;
            double n0, n1, n2;
            if (iss >> n0 >> n1 >> n2) {
                normal.push_back(n0);
                normal.push_back(n1);
                normal.push_back(n2);
            } else {
                throw std::runtime_error("file format error!");
            }
            Eigen::Vector3d normalized_n = Eigen::Map<Eigen::Vector3d>(&normal[0], 3).normalized();
            normal[0] = normalized_n.x();
            normal[1] = normalized_n.y();
            normal[2] = normalized_n.z();
            vertices_normal.push_back(normal);
        }; break;
        case VP: {
            std::vector<double> parameters;
            double p0, p1, p2;
            if (iss >> p0 >> p1 >> p2) {
                parameters.push_back(p0);
                parameters.push_back(p1);
                parameters.push_back(p2);
            } else {
                throw std::runtime_error("file format error!");
            }
            vertices_parameter.push_back(parameters);
        }; break;
        case COMMENT:
        default: continue;
        }
    }
}
} // namespace wmtk