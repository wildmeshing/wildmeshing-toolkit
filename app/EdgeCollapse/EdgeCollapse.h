#include <wmtk/TriMesh.h>
#include <wmtk/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

class EdgeCollapse : public wmtk::TriMesh
{
private:
    std::vector<Eigen::Vector3d> m_vertex_positions;

    EdgeCollapse(std::vector<Eigen::Vector3d> _m_vertex_positions)
        : m_vertex_position(_m_vertex_position)
    {}

    ~EdgeCollapse() {}

    // get the quadrix in form of an array of 10 floating point numbers
    std::array<double, 10> compute_Q_f(wmtk::TriMesh& m, size_t fid);

    std::array<double, 10> compute_Q_v(wmtk::TriMesh& m, size_t vid);

    void collapse_queue(wmtk::TriMesh& m);
}