#include "IndexSimplexMapper.hpp"
#include <wmtk/utils/edgemesh_topology_initialization.h>
#include <wmtk/utils/tetmesh_topology_initialization.h>
#include <wmtk/utils/trimesh_topology_initialization.h>
#include <algorithm>
#include <set>
#include <wmtk/Mesh.hpp>
#include <wmtk/utils/EigenMatrixWriter.hpp>

namespace wmtk::utils::internal {
namespace {
template <int D, int E>
std::vector<std::array<int64_t, E>> get_simplices(std::array<int64_t, D> s)
{
    static_assert(E <= D);
    std::set<std::array<int64_t, E>> F;

    std::array<int64_t, E> r;

    // extract the first E elements of each permutation of s
    std::sort(s.begin(), s.end());
    do {
        std::copy(s.begin(), s.begin() + E, r.begin());
        std::sort(r.begin(), r.end());
        // append to the simplex set
        F.emplace(r);
    } while (std::next_permutation(s.begin(), s.end()));
    return std::vector<std::array<int64_t, E>>{F.begin(), F.end()};
}
template <int D, int E>
std::vector<std::array<int64_t, E>> get_simplices(const std::vector<std::array<int64_t, D>>& S)
{
    static_assert(E <= D);
    std::set<std::array<int64_t, E>> F;

    // go through every simplex
    for (const std::array<int64_t, D>& s : S) {
        auto ss = get_simplices<D, E>(s);
        std::copy(ss.begin(), ss.end(), std::inserter(F, F.end()));
    }
    return std::vector<std::array<int64_t, E>>{F.begin(), F.end()};
}

template <int D>
std::vector<std::array<int64_t, D>> from_eigen(Eigen::Ref<const RowVectors<int64_t, D>>& S)
{
    std::vector<std::array<int64_t, D>> r(S.rows());
    for (int j = 0; j < S.rows(); ++j) {
        Eigen::Map<Eigen::RowVector<int64_t, D>>(r[j].data()) = S.row(j);
    }
    return r;
}
auto get_simplices(const Mesh& m)
{
    EigenMatrixWriter writer;
    m.serialize(writer);
    return writer.get_simplex_vertex_matrix();
}

} // namespace

IndexSimplexMapper::IndexSimplexMapper(const Mesh& mesh)
    : IndexSimplexMapper(get_simplices(mesh))
{}
IndexSimplexMapper::IndexSimplexMapper(Eigen::Ref<const MatrixXl> S)
{
    switch (S.cols()) {
    case 2: initialize_edge_mesh(S); break;
    case 3: initialize_tri_mesh(S); break;
    case 4: initialize_tet_mesh(S); break;
    default: assert(false); break;
    }
}
void IndexSimplexMapper::initialize_edge_mesh(Eigen::Ref<const RowVectors2l> S)
{
    auto SV = from_eigen<2>(S);
    m_E = make_map<2>(SV);
    m_V = make_child_map<2, 1>(SV);
}
void IndexSimplexMapper::initialize_tri_mesh(Eigen::Ref<const RowVectors3l> S)
{
    auto SV = from_eigen<3>(S);
    m_F = make_map<3>(SV);
    m_E = make_child_map<3, 2>(SV);
    m_V = make_child_map<3, 1>(SV);
}
void IndexSimplexMapper::initialize_tet_mesh(Eigen::Ref<const RowVectors4l> S)
{
    auto SV = from_eigen<4>(S);
    m_T = make_map<4>(SV);
    m_F = make_child_map<4, 3>(SV);
    m_E = make_child_map<4, 2>(SV);
    m_V = make_child_map<4, 1>(SV);
}
template <int Dim, int ChildDim>
std::map<std::array<int64_t, ChildDim>, int64_t> IndexSimplexMapper::make_child_map(
    std::vector<std::array<int64_t, Dim>> S)
{
    auto C = get_simplices<Dim, ChildDim>(S);
    return make_map<ChildDim>(C);
}
template <int Dim>
std::map<std::array<int64_t, Dim>, int64_t> IndexSimplexMapper::make_map(
    std::vector<std::array<int64_t, Dim>> S)
{
    std::map<std::array<int64_t, Dim>, int64_t> mp;
    for (auto a : S) {
        std::sort(a.begin(), a.end());
        size_t cur_size = mp.size();
        // std::map says if element already existed no element is added
        mp.emplace(a, cur_size);
    }
    return mp;
}

template <int Dim>
int64_t IndexSimplexMapper::get_index(
    std::array<int64_t, Dim> s,
    const std::map<std::array<int64_t, Dim>, int64_t>& mp)
{
    std::sort(s.begin(), s.end());
    return mp.at(s);
}
template <int Dim>
int64_t get_index(
    Eigen::Ref<const RowVector<int64_t, Dim>> s,
    const std::map<std::array<int64_t, Dim>, int64_t>& mp)
{
    std::array<int64_t, Dim> a;
    Eigen::Map<RowVector<int64_t, Dim>>(a.data()) = s;
    return get_index(a, mp);
}
} // namespace wmtk::utils::internal
