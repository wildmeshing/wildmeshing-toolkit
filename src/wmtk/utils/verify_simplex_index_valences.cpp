
#include <fmt/ranges.h>
#include <set>
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleInspector.hpp>
#include <wmtk/utils/primitive_range.hpp>
#include "internal/IndexSimplexMapper.hpp"
namespace wmtk::utils {
namespace {

template <int Dim>
std::array<int64_t, Dim + 1>
indices(const Mesh& m, const internal::IndexSimplexMapper& mapper, const simplex::Simplex& s)
{
    assert(Dim == get_primitive_type_id(s.primitive_type()));

    std::array<int64_t, Dim + 1> r;
    if (s.primitive_type() == PrimitiveType::Vertex) {
        simplex::IdSimplex id = m.get_id_simplex(s);
        r[0] = mapper.id(id);

    } else {
        auto simps = simplex::faces_single_dimension(m, s, PrimitiveType::Vertex);
        auto cof = simps.simplex_vector(PrimitiveType::Vertex);
        assert(cof.size() == Dim + 1);
        for (size_t j = 0; j < Dim + 1; ++j) {
            simplex::IdSimplex id = m.get_id_simplex(cof[j]);
            r[j] = mapper.id(id);
        }
    }

    return r;
}
template <PrimitiveType mesh_pt, PrimitiveType pt>
bool verify_simplex_index_valences(const Mesh& m, const internal::IndexSimplexMapper& mapper)
{
    constexpr static int meshD = get_primitive_type_id(mesh_pt);
    constexpr static int D = get_primitive_type_id(pt);


    std::vector<std::set<int64_t>> cofaces(mapper.simplices<D>().size());
    for (size_t j = 0; j < mapper.simplices<meshD>().size(); ++j) {
        for (const auto& face_index : mapper.faces<meshD, D>(j)) {
            cofaces[face_index].emplace(j);
        }
    }

    if (mesh_pt == pt + 1) {
        for (size_t j = 0; j < cofaces.size(); ++j) {
            const auto& cof = cofaces[j];
            if (cof.size() > 2) {
                wmtk::logger().warn(fmt::format(
                    "More than 2 {}-cofaces (facet indices={}) for a boundary {}-simplex [{}]",
                    D,
                    fmt::join(cof, ","),
                    meshD,
                    fmt::join(mapper.simplices<D>()[j], ",")));
                return false;
            }
        }
    }


    for (const Tuple& t : m.get_all(pt)) {
        simplex::Simplex s(pt, t);

        auto cof = simplex::cofaces_single_dimension(m, s, mesh_pt).simplex_vector();

        std::array<int64_t, D + 1> i = indices<D>(m, mapper, s);
        const auto& cof2 = cofaces[mapper.get_index<D>(i)];
        wmtk::logger().debug("Looking at {}-simplex {} on a {}-mesh", D, fmt::join(i, ","), meshD);


        if (cof.size() != cof2.size()) {
            wmtk::logger().warn(
                "Cofaces size mismatch on {}-simplex {}, {}-mesh got (len{}), indices got [{}] "
                "(len{})",
                D,
                fmt::join(i, ","),
                meshD,
                cof.size(),
                fmt::join(cof2, ","),
                cof2.size());
            return false;
        }
        for (const simplex::Simplex& facet : cof) {
            assert(facet.primitive_type() == mesh_pt);

            simplex::IdSimplex id = m.get_id_simplex(facet);
            int64_t index = mapper.id(id);

            if (cof2.find(index) == cof2.end()) {
                wmtk::logger().warn(
                    "Cofaces mismatch on simplex {}, mesh simplex {} was not found in indices list "
                    " [{}] "
                    "(len{})",
                    fmt::join(i, ","),
                    index,
                    fmt::join(cof2, ","),
                    cof2.size());
                return false;
            }
        }
    }
    return true;
}
} // namespace

bool verify_simplex_index_valences(const Mesh& m)
{
    internal::IndexSimplexMapper mapper(m);


    PrimitiveType top = m.top_simplex_type();
    auto run = [&](const auto& d) -> bool {
        using D = std::decay_t<decltype(d)>;

        constexpr static PrimitiveType pt = D::value;
        bool ok = true;
        if constexpr (pt >= PrimitiveType::Vertex) {
            ok &= verify_simplex_index_valences<pt, PrimitiveType::Vertex>(m, mapper);
        }
        if constexpr (pt >= PrimitiveType::Edge) {
            ok &= verify_simplex_index_valences<pt, PrimitiveType::Edge>(m, mapper);
        }
        if constexpr (pt >= PrimitiveType::Triangle) {
            ok &= verify_simplex_index_valences<pt, PrimitiveType::Triangle>(m, mapper);
        }
        if constexpr (pt >= PrimitiveType::Tetrahedron) {
            ok &= verify_simplex_index_valences<pt, PrimitiveType::Tetrahedron>(m, mapper);
        }
        return ok;
    };
    switch (top) {
    case PrimitiveType::Vertex:
        return true;
        // return run(std::integral_constant<PrimitiveType, PrimitiveType::Vertex>{});
    case PrimitiveType::Edge:
        return run(std::integral_constant<PrimitiveType, PrimitiveType::Edge>{});
    case PrimitiveType::Triangle:
        return run(std::integral_constant<PrimitiveType, PrimitiveType::Triangle>{});
    case PrimitiveType::Tetrahedron:
        return run(std::integral_constant<PrimitiveType, PrimitiveType::Tetrahedron>{});
    default: assert(false); return {};
    }
}
} // namespace wmtk::utils
