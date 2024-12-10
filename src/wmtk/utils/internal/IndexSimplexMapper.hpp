#pragma once
#include <map>
#include <wmtk/Types.hpp>

namespace wmtk {
class Mesh;
}
namespace wmtk::utils::internal {
class IndexSimplexMapper
{
public:
    IndexSimplexMapper(Eigen::Ref<const MatrixXl> S);
    IndexSimplexMapper(const Mesh& mesh);

private:
    void initialize_edge_mesh(Eigen::Ref<const RowVectors2l> S);
    void initialize_tri_mesh(Eigen::Ref<const RowVectors3l> S);
    void initialize_tet_mesh(Eigen::Ref<const RowVectors4l> S);


    template <int Dim>
    static int64_t get_index(
        std::array<int64_t, Dim> s,
        const std::map<std::array<int64_t, Dim>, int64_t>& mp);
    template <int Dim>
    static int64_t get_index(
        Eigen::Ref<const RowVector<int64_t, Dim>> S,
        const std::map<std::array<int64_t, Dim>, int64_t>& mp);

    template <int Dim, int ChildDim>
    static std::map<std::array<int64_t, ChildDim>, int64_t> make_child_map(
        std::vector<std::array<int64_t, Dim>> S);
    template <int Dim>
    static std::map<std::array<int64_t, Dim>, int64_t> make_map(
        std::vector<std::array<int64_t, Dim>> S);


    std::map<std::array<int64_t, 1>, int64_t> m_V;
    std::map<std::array<int64_t, 2>, int64_t> m_E;
    std::map<std::array<int64_t, 3>, int64_t> m_F;
    std::map<std::array<int64_t, 4>, int64_t> m_T;
};
} // namespace wmtk::utils::internal
