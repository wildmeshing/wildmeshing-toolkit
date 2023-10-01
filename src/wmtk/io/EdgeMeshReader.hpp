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
    void read(Eigen::Matrix<long, -1, -1>& E, Eigen::MatrixXd& V);
    void read_obj(
        std::vector<std::pair<long, long>>& edges,
        std::vector<std::vector<double>>& vertices);
    void read_off(
        std::vector<std::pair<long, long>>& edges,
        std::vector<std::vector<double>>& vertices);

    std::string m_filename;
    file_type m_type;
};
} // namespace wmtk