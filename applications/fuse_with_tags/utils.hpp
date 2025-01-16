#pragma once
#include <Eigen/Core>
#include <set>

#include <map>
namespace wmtk::components::multimesh {
class MeshCollection;
}
#include <Eigen/Core>

template <typename T>
struct EigenMesh
{
    Eigen::Index start() const { return m_start; }
    Eigen::Index end() const { return m_start + M.rows(); }

    Eigen::Index m_start = 0;
    Eigen::MatrixX<T> M;

    template <typename D>
    void assign(Eigen::MatrixBase<D>& m) const
    {
        m.block(start(), 0, M.rows(), M.cols()) = M;
        // m(Eigen::seq(start(), end()), Eigen::all) = M;
    }
};
struct EigenMeshes
{
    EigenMesh<double> V;
    EigenMesh<int64_t> F;

    std::vector<std::set<int64_t>> VF;

    void compute_vf();

    bool in_v_range(int64_t index ) const {
        return index >= V.start() && index < V.end();
    }
    bool in_f_range(int64_t index ) const {
        return index >= F.M.minCoeff()  && index <= F.M.maxCoeff();
    }
};

std::map<std::string, EigenMeshes> get_meshes(
    const wmtk::components::multimesh::MeshCollection& mc,
    const std::string_view& position_attribute_name);
