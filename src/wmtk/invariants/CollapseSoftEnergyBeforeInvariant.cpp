#include "CollapseSoftEnergyBeforeInvariant.hpp"
#include <wmtk/simplex/utils/SimplexComparisons.hpp>

#include <wmtk/function/utils/amips.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/simplex/utils/SimplexComparisons.hpp>
#include <wmtk/utils/orient.hpp>

namespace wmtk::invariants {

CollapseSoftEnergyBeforeInvariant::CollapseSoftEnergyBeforeInvariant(
    const Mesh& m,
    const attribute::TypedAttributeHandle<Rational>& coordinate,
    const attribute::TypedAttributeHandle<double>& energy,
    const attribute::TypedAttributeHandle<double>& edge_length,
    const attribute::TypedAttributeHandle<double>& target_edge_length,
    int64_t collapse_type,
    double eps)
    : Invariant(m, true, false, false)
    , m_coordinate_handle(coordinate)
    , m_energy_handle(energy)
    , m_edge_length_handle(edge_length)
    , m_target_edge_length_handle(target_edge_length)
    , m_collapse_type(collapse_type)
    , m_eps(eps)
{}

bool CollapseSoftEnergyBeforeInvariant::before(const simplex::Simplex& s) const
{
    constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Triangle;
    constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;

    if (mesh().top_simplex_type() == PT) {
        auto position_accessor = mesh().create_const_accessor(m_coordinate_handle);

        auto amips_accessor = mesh().create_const_accessor(m_energy_handle);
        auto edge_length_accessor = mesh().create_const_accessor(m_edge_length_handle);
        auto target_edge_length_accessor =
            mesh().create_const_accessor(m_target_edge_length_handle);

        const double current_length = edge_length_accessor.const_scalar_attribute(s.tuple());
        const double target_length = target_edge_length_accessor.const_scalar_attribute(s.tuple());
        const bool mini_edge = current_length <= target_length * m_eps;

        double old_energy_max = std::numeric_limits<double>::lowest();
        double new_energy_max = std::numeric_limits<double>::lowest();

        switch (m_collapse_type) {
        case 0: {
            const Tuple v0 = s.tuple();
            const Tuple v1 = mesh().switch_tuple(v0, PV);
            const auto v1_pos = position_accessor.const_vector_attribute(v1);

            // TODO: check sort and clean
            const auto& v0_incident_tets =
                simplex::top_dimension_cofaces_tuples(mesh(), simplex::Simplex::vertex(mesh(), v0));
            const auto& v1_incident_tets =
                simplex::top_dimension_cofaces_tuples(mesh(), simplex::Simplex::vertex(mesh(), v1));

            // compute energy for v1 incident tets
            for (const Tuple& t : v1_incident_tets) {
                const std::array<Tuple, 4> lv = {
                    {t,
                     mesh().switch_tuple(t, PV),
                     mesh().switch_tuples(t, {PE, PV}),
                     mesh().switch_tuples(t, {PF, PE, PV})}};

                // compute old max
                old_energy_max = std::max(old_energy_max, amips_accessor.const_scalar_attribute(t));

                // compute new energy
                const simplex::Simplex v0_s(mesh(), PV, v0);

                if (simplex::utils::SimplexComparisons::equal(
                        mesh(),
                        simplex::Simplex::vertex(mesh(), lv[1]),
                        v0_s) ||
                    simplex::utils::SimplexComparisons::equal(
                        mesh(),
                        simplex::Simplex::vertex(mesh(), lv[2]),
                        v0_s) ||
                    simplex::utils::SimplexComparisons::equal(
                        mesh(),
                        simplex::Simplex::vertex(mesh(), lv[3]),
                        v0_s)) {
                    // skip if incident both vertices
                    continue;
                }

                // position after the collapse
                const std::array<Eigen::Vector<Rational, 3>, 4> lv_pos = {
                    {position_accessor.const_vector_attribute(v0), // v1 collapsed to v0
                     position_accessor.const_vector_attribute(lv[1]),
                     position_accessor.const_vector_attribute(lv[2]),
                     position_accessor.const_vector_attribute(lv[3])}};

                // check inversion, any new tet inverted return false
                bool is_ccw_tuple = mesh().is_ccw(lv[0]);
                if (is_ccw_tuple) {
                    if (wmtk::utils::wmtk_orient3d(lv_pos[3], lv_pos[0], lv_pos[1], lv_pos[2]) <=
                        0) {
                        return false;
                    }
                } else {
                    if (wmtk::utils::wmtk_orient3d(lv_pos[3], lv_pos[0], lv_pos[2], lv_pos[1]) <=
                        0) {
                        return false;
                    }
                }

                double new_energy;
                if (is_ccw_tuple) {
                    new_energy = wmtk::function::utils::Tet_AMIPS_energy(
                        {{lv_pos[3][0].to_double(),
                          lv_pos[3][1].to_double(),
                          lv_pos[3][2].to_double(),
                          lv_pos[0][0].to_double(),
                          lv_pos[0][1].to_double(),
                          lv_pos[0][2].to_double(),
                          lv_pos[1][0].to_double(),
                          lv_pos[1][1].to_double(),
                          lv_pos[1][2].to_double(),
                          lv_pos[2][0].to_double(),
                          lv_pos[2][1].to_double(),
                          lv_pos[2][2].to_double()}});
                } else {
                    new_energy = wmtk::function::utils::Tet_AMIPS_energy(
                        {{lv_pos[3][0].to_double(),
                          lv_pos[3][1].to_double(),
                          lv_pos[3][2].to_double(),
                          lv_pos[0][0].to_double(),
                          lv_pos[0][1].to_double(),
                          lv_pos[0][2].to_double(),
                          lv_pos[2][0].to_double(),
                          lv_pos[2][1].to_double(),
                          lv_pos[2][2].to_double(),
                          lv_pos[1][0].to_double(),
                          lv_pos[1][1].to_double(),
                          lv_pos[1][2].to_double()}});
                }
                new_energy_max = std::max(new_energy_max, new_energy);
            }

            // passed all inversion check, return true if v1 is not rounded
            for (int64_t i = 0; i < 3; ++i) {
                // if the collapse from vertex is not rounded, return true anyway
                if (!v1_pos[i].is_rounded()) return true;
            }

            // collapse a tiny edge anyway
            if (mini_edge) {
                return true;
            }

            // compute energy for v0 incident, old and new are the same because v0 is not moved
            for (const Tuple& t : v0_incident_tets) {
                const std::array<Tuple, 4> lv = {
                    {t,
                     mesh().switch_tuple(t, PV),
                     mesh().switch_tuples(t, {PE, PV}),
                     mesh().switch_tuples(t, {PF, PE, PV})}};

                // skip tets incident the edge, old is already computed above
                const simplex::Simplex v1_s(mesh(), PV, v1);
                if (simplex::utils::SimplexComparisons::equal(
                        mesh(),
                        simplex::Simplex::vertex(mesh(), lv[1]),
                        v1_s) ||
                    simplex::utils::SimplexComparisons::equal(
                        mesh(),
                        simplex::Simplex::vertex(mesh(), lv[2]),
                        v1_s) ||
                    simplex::utils::SimplexComparisons::equal(
                        mesh(),
                        simplex::Simplex::vertex(mesh(), lv[3]),
                        v1_s)) {
                    // skip if incident both vertices
                    continue;
                }

                // compute old max
                const double energy = amips_accessor.const_scalar_attribute(t);
                old_energy_max = std::max(old_energy_max, energy);

                // compute new energy
                new_energy_max = std::max(new_energy_max, energy);
            }

            break;
        }
        case 1: {
            const Tuple v0 = s.tuple();
            const Tuple v1 = mesh().switch_tuple(v0, PV);
            const auto v0_pos = position_accessor.const_vector_attribute(v0);

            // TODO: check sort and clean
            const auto& v0_incident_tets =
                simplex::top_dimension_cofaces_tuples(mesh(), simplex::Simplex::vertex(mesh(), v0));
            const auto& v1_incident_tets =
                simplex::top_dimension_cofaces_tuples(mesh(), simplex::Simplex::vertex(mesh(), v1));

            // compute energy for v0 incident tets
            for (const Tuple& t : v0_incident_tets) {
                const std::array<Tuple, 4> lv = {
                    {t,
                     mesh().switch_tuple(t, PV),
                     mesh().switch_tuples(t, {PE, PV}),
                     mesh().switch_tuples(t, {PF, PE, PV})}};

                // compute old max
                old_energy_max = std::max(old_energy_max, amips_accessor.const_scalar_attribute(t));

                // compute new energy
                const simplex::Simplex v1_s(mesh(), PV, v1);
                if (simplex::utils::SimplexComparisons::equal(
                        mesh(),
                        simplex::Simplex::vertex(mesh(), lv[1]),
                        v1_s) ||
                    simplex::utils::SimplexComparisons::equal(
                        mesh(),
                        simplex::Simplex::vertex(mesh(), lv[2]),
                        v1_s) ||
                    simplex::utils::SimplexComparisons::equal(
                        mesh(),
                        simplex::Simplex::vertex(mesh(), lv[3]),
                        v1_s)) {
                    // skip if incident both vertices
                    continue;
                }

                // position after the collapse
                const std::array<Eigen::Vector<Rational, 3>, 4> lv_pos = {
                    {position_accessor.const_vector_attribute(v1), // v0 collapsed to v1
                     position_accessor.const_vector_attribute(lv[1]),
                     position_accessor.const_vector_attribute(lv[2]),
                     position_accessor.const_vector_attribute(lv[3])}};

                // check inversion, any new tet inverted return false
                bool is_ccw_tuple = mesh().is_ccw(lv[0]);
                if (is_ccw_tuple) {
                    if (wmtk::utils::wmtk_orient3d(lv_pos[3], lv_pos[0], lv_pos[1], lv_pos[2]) <=
                        0) {
                        return false;
                    }
                } else {
                    if (wmtk::utils::wmtk_orient3d(lv_pos[3], lv_pos[0], lv_pos[2], lv_pos[1]) <=
                        0) {
                        return false;
                    }
                }

                double new_energy;
                if (is_ccw_tuple) {
                    new_energy = wmtk::function::utils::Tet_AMIPS_energy(
                        {{lv_pos[3][0].to_double(),
                          lv_pos[3][1].to_double(),
                          lv_pos[3][2].to_double(),
                          lv_pos[0][0].to_double(),
                          lv_pos[0][1].to_double(),
                          lv_pos[0][2].to_double(),
                          lv_pos[1][0].to_double(),
                          lv_pos[1][1].to_double(),
                          lv_pos[1][2].to_double(),
                          lv_pos[2][0].to_double(),
                          lv_pos[2][1].to_double(),
                          lv_pos[2][2].to_double()}});
                } else {
                    new_energy = wmtk::function::utils::Tet_AMIPS_energy(
                        {{lv_pos[3][0].to_double(),
                          lv_pos[3][1].to_double(),
                          lv_pos[3][2].to_double(),
                          lv_pos[0][0].to_double(),
                          lv_pos[0][1].to_double(),
                          lv_pos[0][2].to_double(),
                          lv_pos[2][0].to_double(),
                          lv_pos[2][1].to_double(),
                          lv_pos[2][2].to_double(),
                          lv_pos[1][0].to_double(),
                          lv_pos[1][1].to_double(),
                          lv_pos[1][2].to_double()}});
                }
                new_energy_max = std::max(new_energy_max, new_energy);
            }

            // passed all inversion check, return true if v1 is not rounded
            for (int64_t i = 0; i < 3; ++i) {
                // if the collapse from vertex is not rounded, return true anyway
                if (!v0_pos[i].is_rounded()) return true;
            }

            // collapse a tiny edge anyway
            if (mini_edge) {
                return true;
            }

            // compute energy for v1 incident, old and new are the same because v0 is not moved
            for (const Tuple& t : v1_incident_tets) {
                const std::array<Tuple, 4> lv = {
                    {t,
                     mesh().switch_tuple(t, PV),
                     mesh().switch_tuples(t, {PE, PV}),
                     mesh().switch_tuples(t, {PF, PE, PV})}};

                // skip tets incident the edge, old is already computed above
                const simplex::Simplex v0_s(mesh(), PV, v0);
                if (simplex::utils::SimplexComparisons::equal(
                        mesh(),
                        simplex::Simplex::vertex(mesh(), lv[1]),
                        v0_s) ||
                    simplex::utils::SimplexComparisons::equal(
                        mesh(),
                        simplex::Simplex::vertex(mesh(), lv[2]),
                        v0_s) ||
                    simplex::utils::SimplexComparisons::equal(
                        mesh(),
                        simplex::Simplex::vertex(mesh(), lv[3]),
                        v0_s)) {
                    // skip if incident both vertices
                    continue;
                }

                // compute old max
                const double energy = amips_accessor.const_scalar_attribute(t);
                old_energy_max = std::max(old_energy_max, energy);

                // compute new energy
                new_energy_max = std::max(new_energy_max, energy);
            }

            break;
        }

        case 2: {
            throw std::runtime_error("not implemented");
        }

        default: throw std::runtime_error("Invalid collapse type");
        }

        return new_energy_max <= old_energy_max;

    } else if (mesh().top_simplex_type() == PF) {
        // TODO: not implement yet
        return true;
    }

    return true;
}

} // namespace wmtk::invariants
