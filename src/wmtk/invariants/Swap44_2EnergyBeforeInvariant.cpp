#include "Swap44_2EnergyBeforeInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/function/utils/amips.hpp>
#include "predicates.h"

namespace wmtk {
Swap44_2EnergyBeforeInvariant::Swap44_2EnergyBeforeInvariant(
    const Mesh& m,
    const TypedAttributeHandle<double>& coordinate)
    : Invariant(m, true, false, false)
    , m_coordinate_handle(coordinate)
{}

bool Swap44_2EnergyBeforeInvariant::before(const simplex::Simplex& t) const
{
    constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Triangle;
    constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;

    auto accessor = mesh().create_const_accessor(m_coordinate_handle);

    // get the coords of the vertices
    // input edge end points
    const Tuple v0 = t.tuple();
    const Tuple v1 = mesh().switch_tuple(v0, PV);
    // other four vertices
    const Tuple v2 = mesh().switch_tuples(v0, {PE, PV});
    const Tuple v3 = mesh().switch_tuples(v0, {PF, PE, PV});
    const Tuple v4 = mesh().switch_tuples(v0, {PF, PT, PF, PE, PV});
    const Tuple v5 = mesh().switch_tuples(v0, {PT, PF, PE, PV});

    std::array<Eigen::Vector3d, 6> positions = {
        {accessor.const_vector_attribute(v0),
         accessor.const_vector_attribute(v1),
         accessor.const_vector_attribute(v2),
         accessor.const_vector_attribute(v3),
         accessor.const_vector_attribute(v4),
         accessor.const_vector_attribute(v5)}};

    std::array<std::array<int, 4>, 4> old_tets = {
        {{{0, 1, 2, 3}}, {{0, 1, 3, 4}}, {{0, 1, 4, 5}}, {{0, 1, 5, 2}}}};
    std::array<std::array<int, 4>, 4> new_tets = {
        {{{3, 5, 0, 2}}, {{3, 5, 2, 1}}, {{3, 5, 1, 4}}, {{3, 5, 4, 0}}}};

    double old_energy_sum = 0;
    double old_energy_max = std::numeric_limits<double>::lowest();
    double new_energy_sum = 0;
    double new_energy_max = std::numeric_limits<double>::lowest();

    for (int i = 0; i < 4; ++i) {
        if (orient3d(
                positions[old_tets[i][0]].data(),
                positions[old_tets[i][1]].data(),
                positions[old_tets[i][2]].data(),
                positions[old_tets[i][3]].data()) > 0) {
            auto energy = wmtk::function::utils::Tet_AMIPS_energy({{
                positions[old_tets[i][0]][0],
                positions[old_tets[i][0]][1],
                positions[old_tets[i][0]][2],
                positions[old_tets[i][1]][0],
                positions[old_tets[i][1]][1],
                positions[old_tets[i][1]][2],
                positions[old_tets[i][2]][0],
                positions[old_tets[i][2]][1],
                positions[old_tets[i][2]][2],
                positions[old_tets[i][3]][0],
                positions[old_tets[i][3]][1],
                positions[old_tets[i][3]][2],
            }});


            old_energy_sum += energy;
            if (energy > old_energy_max) old_energy_max = energy;
        } else {
            auto energy = wmtk::function::utils::Tet_AMIPS_energy({{
                positions[old_tets[i][1]][0],
                positions[old_tets[i][1]][1],
                positions[old_tets[i][1]][2],
                positions[old_tets[i][0]][0],
                positions[old_tets[i][0]][1],
                positions[old_tets[i][0]][2],
                positions[old_tets[i][2]][0],
                positions[old_tets[i][2]][1],
                positions[old_tets[i][2]][2],
                positions[old_tets[i][3]][0],
                positions[old_tets[i][3]][1],
                positions[old_tets[i][3]][2],
            }});


            old_energy_sum += energy;
            if (energy > old_energy_max) old_energy_max = energy;
        }

        if (orient3d(
                positions[new_tets[i][0]].data(),
                positions[new_tets[i][1]].data(),
                positions[new_tets[i][2]].data(),
                positions[new_tets[i][3]].data()) > 0) {
            auto energy = wmtk::function::utils::Tet_AMIPS_energy({{
                positions[new_tets[i][0]][0],
                positions[new_tets[i][0]][1],
                positions[new_tets[i][0]][2],
                positions[new_tets[i][1]][0],
                positions[new_tets[i][1]][1],
                positions[new_tets[i][1]][2],
                positions[new_tets[i][2]][0],
                positions[new_tets[i][2]][1],
                positions[new_tets[i][2]][2],
                positions[new_tets[i][3]][0],
                positions[new_tets[i][3]][1],
                positions[new_tets[i][3]][2],
            }});


            new_energy_sum += energy;
            if (energy > new_energy_max) new_energy_max = energy;
        } else {
            auto energy = wmtk::function::utils::Tet_AMIPS_energy({{
                positions[new_tets[i][1]][0],
                positions[new_tets[i][1]][1],
                positions[new_tets[i][1]][2],
                positions[new_tets[i][0]][0],
                positions[new_tets[i][0]][1],
                positions[new_tets[i][0]][2],
                positions[new_tets[i][2]][0],
                positions[new_tets[i][2]][1],
                positions[new_tets[i][2]][2],
                positions[new_tets[i][3]][0],
                positions[new_tets[i][3]][1],
                positions[new_tets[i][3]][2],
            }});


            new_energy_sum += energy;
            if (energy > new_energy_max) new_energy_max = energy;
        }
    }

    return old_energy_max > new_energy_max;
}

} // namespace wmtk
