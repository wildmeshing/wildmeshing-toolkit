#include "IndexSimplexMapper.hpp"
#include <wmtk/utils/edgemesh_topology_initialization.h>
#include <wmtk/utils/tetmesh_topology_initialization.h>
#include <wmtk/utils/trimesh_topology_initialization.h>
#include <algorithm>
#include <set>
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/IdSimplex.hpp>
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

int64_t IndexSimplexMapper::id(const simplex::IdSimplex& id)
{
    return id.index();
}
void IndexSimplexMapper::initialize_edge_mesh(Eigen::Ref<const RowVectors2l> S)
{
    m_E = from_eigen<2>(S);
    m_E_map = make_map<2>(m_E);
    m_V_map = make_child_map<2, 1>(m_E);
    update_simplices();
}
void IndexSimplexMapper::initialize_tri_mesh(Eigen::Ref<const RowVectors3l> S)
{
    m_F = from_eigen<3>(S);
    m_F_map = make_map<3>(m_F);
    m_E_map = make_child_map<3, 2>(m_F);
    m_V_map = make_child_map<3, 1>(m_F);
    update_simplices();
}
void IndexSimplexMapper::initialize_tet_mesh(Eigen::Ref<const RowVectors4l> S)
{
    m_T = from_eigen<4>(S);
    m_T_map = make_map<4>(m_T);
    m_F_map = make_child_map<4, 3>(m_T);
    m_E_map = make_child_map<4, 2>(m_T);
    m_V_map = make_child_map<4, 1>(m_T);

    update_simplices();
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
void IndexSimplexMapper::update_simplices()
{
    auto update = [](const auto& mp, auto& vec) {
        vec.resize(mp.size());

        for (const auto& [arr, ind] : mp) {
            vec[ind] = arr;
        }
    };

    update(m_V_map, m_V);
    update(m_E_map, m_E);
    update(m_F_map, m_F);
    update(m_T_map, m_T);
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

template <int Dim>
int64_t IndexSimplexMapper::get_index(const std::array<int64_t, Dim + 1>& s) const
{
    return get_index<Dim + 1>(s, simplex_map<Dim>());
}

template <int Dim, int ChildDim>
std::vector<int64_t> IndexSimplexMapper::faces(size_t index) const
{
    if constexpr (Dim < ChildDim) {
        return {};
    } else {
        auto s = simplices<Dim>().at(index);
        auto faces = get_simplices<Dim + 1, ChildDim + 1>(s);
        std::vector<int64_t> r;
        r.reserve(faces.size());
        std::transform(
            faces.begin(),
            faces.end(),
            std::back_inserter(r),
            [&](const std::array<int64_t, ChildDim + 1>& d) {
                return simplex_map<ChildDim>().at(d);
            });
        return r;
    }
}
std::vector<int64_t> IndexSimplexMapper::faces(size_t index, int8_t dim, int8_t child_dim) const
{
    auto run = [&](const auto& d) -> std::vector<int64_t> {
        using D = std::decay_t<decltype(d)>;

        constexpr static int Dim = D::value;
        switch (child_dim) {
        case 0: return faces<Dim, 0>(index);
        case 1: return faces<Dim, 1>(index);
        case 2: return faces<Dim, 2>(index);
        case 3: return faces<Dim, 3>(index);
        default: assert(false); return {};
        }
    };
    switch (dim) {
    case 0: return run(std::integral_constant<int, 0>{});
    case 1: return run(std::integral_constant<int, 1>{});
    case 2: return run(std::integral_constant<int, 2>{});
    case 3: return run(std::integral_constant<int, 3>{});
    default: assert(false); return {};
    }
}

template <int D>
const std::map<std::array<int64_t, D + 1>, int64_t>& IndexSimplexMapper::simplex_map() const
{
    if constexpr (D == 0) {
        return m_V_map;
    } else if constexpr (D == 1) {
        return m_E_map;
    } else if constexpr (D == 2) {
        return m_F_map;
    } else if constexpr (D == 3) {
        return m_T_map;
    }
}
template <int D>
const std::vector<std::array<int64_t, D + 1>>& IndexSimplexMapper::simplices() const
{
    if constexpr (D == 0) {
        return m_V;
    } else if constexpr (D == 1) {
        return m_E;
    } else if constexpr (D == 2) {
        return m_F;
    } else if constexpr (D == 3) {
        return m_T;
    }
}

template int64_t IndexSimplexMapper::get_index<0>(const std::array<int64_t, 1>& s) const;
template int64_t IndexSimplexMapper::get_index<1>(const std::array<int64_t, 2>& s) const;
template int64_t IndexSimplexMapper::get_index<2>(const std::array<int64_t, 3>& s) const;
template int64_t IndexSimplexMapper::get_index<3>(const std::array<int64_t, 4>& s) const;

template const std::map<std::array<int64_t, 0 + 1>, int64_t>& IndexSimplexMapper::simplex_map<0>()
    const;
template const std::vector<std::array<int64_t, 0 + 1>>& IndexSimplexMapper::simplices<0>() const;

template const std::map<std::array<int64_t, 1 + 1>, int64_t>& IndexSimplexMapper::simplex_map<1>()
    const;
template const std::vector<std::array<int64_t, 1 + 1>>& IndexSimplexMapper::simplices<1>() const;
template const std::map<std::array<int64_t, 2 + 1>, int64_t>& IndexSimplexMapper::simplex_map<2>()
    const;
template const std::vector<std::array<int64_t, 2 + 1>>& IndexSimplexMapper::simplices<2>() const;
template const std::map<std::array<int64_t, 3 + 1>, int64_t>& IndexSimplexMapper::simplex_map<3>()
    const;
template const std::vector<std::array<int64_t, 3 + 1>>& IndexSimplexMapper::simplices<3>() const;
} // namespace wmtk::utils::internal
