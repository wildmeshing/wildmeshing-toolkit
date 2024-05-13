#include "Swap56EnergyBeforeInvariantDouble.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/function/utils/amips.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/orient.hpp>

namespace wmtk {
Swap56EnergyBeforeInvariantDouble::Swap56EnergyBeforeInvariantDouble(
    const Mesh& m,
    const attribute::TypedAttributeHandle<double>& coordinate,
    int64_t collapse_index,
    double eps)
    : Invariant(m, true, false, false)
    , m_coordinate_handle(coordinate)
    , m_collapse_index(collapse_index)
    , m_eps(eps)
{}

bool Swap56EnergyBeforeInvariantDouble::before(const simplex::Simplex& t) const
{
    assert(simplex::top_dimension_cofaces(mesh(), t).size() == 5);
    constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Triangle;
    constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;

    auto accessor = mesh().create_const_accessor(m_coordinate_handle);

    const Tuple e0 = t.tuple();
    const Tuple e1 = mesh().switch_tuple(e0, PV);

    std::array<Tuple, 5> v;
    auto iter_tuple = e0;
    for (int64_t i = 0; i < 5; ++i) {
        v[i] = mesh().switch_tuples(iter_tuple, {PE, PV});
        iter_tuple = mesh().switch_tuples(iter_tuple, {PF, PT});
    }
    assert(iter_tuple == e0);

    // five iterable vertices remap to 0-4 by m_collapse_index, 0: m_collapse_index, 5: e0, 6: e1
    std::array<Eigen::Vector3<double>, 7> positions = {
        {accessor.const_vector_attribute(v[(m_collapse_index + 0) % 5]),
         accessor.const_vector_attribute(v[(m_collapse_index + 1) % 5]),
         accessor.const_vector_attribute(v[(m_collapse_index + 2) % 5]),
         accessor.const_vector_attribute(v[(m_collapse_index + 3) % 5]),
         accessor.const_vector_attribute(v[(m_collapse_index + 4) % 5]),
         accessor.const_vector_attribute(e0),
         accessor.const_vector_attribute(e1)}};

    std::array<Eigen::Vector3d, 7> positions_double = {
        {positions[0],
         positions[1],
         positions[2],
         positions[3],
         positions[4],
         positions[5],
         positions[6]}};

    std::array<std::array<int, 4>, 5> old_tets = {
        {{{0, 1, 5, 6}}, {{1, 2, 5, 6}}, {{2, 3, 5, 6}}, {{3, 4, 5, 6}}, {{4, 0, 5, 6}}}};

    std::array<std::array<int, 4>, 6> new_tets = {
        {{{0, 1, 2, 5}},
         {{0, 2, 3, 5}},
         {{0, 3, 4, 5}},
         {{0, 1, 2, 6}},
         {{0, 2, 3, 6}},
         {{0, 3, 4, 6}}}};

    double old_energy_max = std::numeric_limits<double>::lowest();
    double new_energy_max = std::numeric_limits<double>::lowest();

    for (int i = 0; i < 5; ++i) {
        if (utils::wmtk_orient3d(
                positions[old_tets[i][0]],
                positions[old_tets[i][1]],
                positions[old_tets[i][2]],
                positions[old_tets[i][3]]) > 0) {
            auto energy = wmtk::function::utils::Tet_AMIPS_energy({{
                positions_double[old_tets[i][0]][0],
                positions_double[old_tets[i][0]][1],
                positions_double[old_tets[i][0]][2],
                positions_double[old_tets[i][1]][0],
                positions_double[old_tets[i][1]][1],
                positions_double[old_tets[i][1]][2],
                positions_double[old_tets[i][2]][0],
                positions_double[old_tets[i][2]][1],
                positions_double[old_tets[i][2]][2],
                positions_double[old_tets[i][3]][0],
                positions_double[old_tets[i][3]][1],
                positions_double[old_tets[i][3]][2],
            }});


            if (energy > old_energy_max) old_energy_max = energy;
        } else {
            auto energy = wmtk::function::utils::Tet_AMIPS_energy({{
                positions_double[old_tets[i][1]][0],
                positions_double[old_tets[i][1]][1],
                positions_double[old_tets[i][1]][2],
                positions_double[old_tets[i][0]][0],
                positions_double[old_tets[i][0]][1],
                positions_double[old_tets[i][0]][2],
                positions_double[old_tets[i][2]][0],
                positions_double[old_tets[i][2]][1],
                positions_double[old_tets[i][2]][2],
                positions_double[old_tets[i][3]][0],
                positions_double[old_tets[i][3]][1],
                positions_double[old_tets[i][3]][2],
            }});


            if (energy > old_energy_max) old_energy_max = energy;
        }
    }

    for (int i = 0; i < 6; ++i) {
        if (utils::wmtk_orient3d(
                positions[new_tets[i][0]],
                positions[new_tets[i][1]],
                positions[new_tets[i][2]],
                positions[new_tets[i][3]]) > 0) {
            auto energy = wmtk::function::utils::Tet_AMIPS_energy({{
                positions_double[new_tets[i][0]][0],
                positions_double[new_tets[i][0]][1],
                positions_double[new_tets[i][0]][2],
                positions_double[new_tets[i][1]][0],
                positions_double[new_tets[i][1]][1],
                positions_double[new_tets[i][1]][2],
                positions_double[new_tets[i][2]][0],
                positions_double[new_tets[i][2]][1],
                positions_double[new_tets[i][2]][2],
                positions_double[new_tets[i][3]][0],
                positions_double[new_tets[i][3]][1],
                positions_double[new_tets[i][3]][2],
            }});


            if (energy > new_energy_max) new_energy_max = energy;
        } else {
            auto energy = wmtk::function::utils::Tet_AMIPS_energy({{
                positions_double[new_tets[i][1]][0],
                positions_double[new_tets[i][1]][1],
                positions_double[new_tets[i][1]][2],
                positions_double[new_tets[i][0]][0],
                positions_double[new_tets[i][0]][1],
                positions_double[new_tets[i][0]][2],
                positions_double[new_tets[i][2]][0],
                positions_double[new_tets[i][2]][1],
                positions_double[new_tets[i][2]][2],
                positions_double[new_tets[i][3]][0],
                positions_double[new_tets[i][3]][1],
                positions_double[new_tets[i][3]][2],
            }});


            if (energy > new_energy_max) new_energy_max = energy;
        }
    }

    return old_energy_max > new_energy_max * m_eps;
}
} // namespace wmtk