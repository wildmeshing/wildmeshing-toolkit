#include "make_unique.hpp"
#include <algorithm>
#include <wmtk/simplex/internal/SimplexEqualFunctor.hpp>
#include <wmtk/simplex/internal/SimplexLessFunctor.hpp>


namespace wmtk::simplex::utils {
std::vector<Simplex> make_unique(const Mesh& m, const std::vector<Simplex>& s)
{
    std::vector<Simplex> ret = s;

    std::sort(ret.begin(), ret.end(), internal::SimplexLessFunctor{m});
    ret.erase(std::unique(ret.begin(), ret.end(), internal::SimplexEqualFunctor{m}), ret.end());

    return ret;
}
std::vector<Tuple>
make_unique_tuples(const Mesh& m, const std::vector<Tuple>& ts, PrimitiveType primitive)
{
    std::vector<Simplex> simps;
    simps.reserve(ts.size());
    std::transform(ts.begin(), ts.end(), std::back_inserter(simps), [&](const Tuple& t) {
        return Simplex(m, primitive, t);
    });

    simps = make_unique(m, simps);
    std::vector<Tuple> ret;
    ret.reserve(simps.size());

    std::transform(simps.begin(), simps.end(), std::back_inserter(ret), [](const Simplex& s) {
        return s.tuple();
    });
    ;

    return ret;
}
} // namespace wmtk::simplex::utils
