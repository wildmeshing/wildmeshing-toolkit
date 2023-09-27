#include <Eigen/Core>
#include <filesystem>
#include <fstream>

namespace wmtk {
class Mesh;

class EdgeMeshReader
{
public:
    enum file_type { OBJ, OFF };
    EdgeMeshReader(const std::filesystem::path& filename, const file_type type);
    void read(Eigen::Matrix<long, -1, -1>& E, Eigen::MatrixXd& V);

    const std::filesystem::path m_filename;
    file_type m_type;
};
} // namespace wmtk