#include <Eigen/Core>
#include <filesystem>
#include <fstream>

namespace wmtk {
class Mesh;
/*
 * @EdgeMeshReader
 * This class is used to read the .obj files, converting into edges and vertices matrix
 * and then passsing them to EdgeMesh constructor
 *
 * @input
 * filename: file path
 * type: file type, for now, only .obj files are supported
 * @output
 * edges
 * vertices
 * vertices_w
 * vertices_texture
 * vertices_normal
 * vertices_parameter
 **/
class EdgeMeshReader
{
public:
    enum data_type { V, L, VT, VN, VP, COMMENT };
    enum file_type { OBJ, OFF };
    EdgeMeshReader(const std::filesystem::path& filename, const file_type type);
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
        std::vector<std::vector<double>>& vertices_normal,
        std::vector<std::vector<double>>& vertices_parameter);
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

    const std::filesystem::path m_filename;
    file_type m_type;
};
} // namespace wmtk