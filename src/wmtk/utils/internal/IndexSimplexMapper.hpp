#pragma once
#include <map>
#include <wmtk/Types.hpp>

namespace wmtk {
class Mesh;
namespace simplex {
class IdSimplex;
}
} // namespace wmtk
namespace wmtk::utils::internal {
class IndexSimplexMapper
{
public:
    IndexSimplexMapper(Eigen::Ref<const MatrixXl> S);
    IndexSimplexMapper(const Mesh& mesh);


    // TOOD: this is just an array of size nCr, but easier to just say vector for now
    template <int Dim, int ChildDim>
    std::vector<int64_t> faces(size_t index) const;

    std::vector<int64_t> faces(size_t index, int8_t dim, int8_t child_dim) const;

    static int64_t id(const simplex::IdSimplex& s);

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


    std::map<std::array<int64_t, 1>, int64_t> m_V_map;
    std::map<std::array<int64_t, 2>, int64_t> m_E_map;
    std::map<std::array<int64_t, 3>, int64_t> m_F_map;
    std::map<std::array<int64_t, 4>, int64_t> m_T_map;


    void update_simplices();
    std::vector<std::array<int64_t, 1>> m_V;
    std::vector<std::array<int64_t, 2>> m_E;
    std::vector<std::array<int64_t, 3>> m_F;
    std::vector<std::array<int64_t, 4>> m_T;


public:
    template <int D>
    const std::map<std::array<int64_t, D + 1>, int64_t>& simplex_map() const;
    template <int D>
    const std::vector<std::array<int64_t, D + 1>>& simplices() const;

    template <int Dim>
    int64_t get_index(const std::array<int64_t, Dim + 1>& s) const;
};
} // namespace wmtk::utils::internal
