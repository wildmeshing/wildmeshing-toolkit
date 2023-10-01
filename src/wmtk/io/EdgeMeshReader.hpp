#include <Eigen/Core>
#include <filesystem>
#include <fstream>

namespace wmtk {
class Mesh;

class EdgeMeshReader
{
public:
    enum file_type { OBJ, OFF };
    EdgeMeshReader(const std::string& filename, const file_type type);
    void read(
        Eigen::Matrix<long, -1, -1>& edges,
        Eigen::MatrixXd& vertices,
        Eigen::MatrixXd& vertices_w,
        Eigen::MatrixXd& vertices_texture,
        Eigen::MatrixXd& vertices_normal,
        Eigen::MatrixXd& vertices_parameter);
    void read_obj(
        std::vector<std::pair<long, long>>& edges,
        std::vector<std::vector<double>>& vertices,
        std::vector<double>& vertices_w,
        std::vector<std::vector<double>>& vertices_texture,
        std::vector<std::vector<double>> vertices_normal,
        std::vector<std::vector<double>>& vertices_parameter);
    void read_off(
        std::vector<std::pair<long, long>>& edges,
        std::vector<std::vector<double>>& vertices);

    std::string m_filename;
    file_type m_type;
};
} // namespace wmtk