#include <Eigen/Core>
#include <filesystem>
#include <fstream>

namespace wmtk {
class Mesh;

class EdgeMeshReader
{
public:
    enum data_type { V, L, VT, VN, VP, COMMENT };
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
    data_type str_to_type(std::string str)
    {
        if (str == "v") {
            return V;
        } else if (str == "l") {
            return L;
        } else if (str == "vt") {
            return VT;
        } else if (str == "vn") {
            return VN;
        } else if (str == "vp") {
            return VP;
        } else {
            return COMMENT;
        }
    }

    std::string m_filename;
    file_type m_type;
};
} // namespace wmtk