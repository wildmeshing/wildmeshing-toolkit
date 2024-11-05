#include "CollapseEnergyBeforeInvariant.hpp"
#include <wmtk/simplex/utils/SimplexComparisons.hpp>

#include <wmtk/function/utils/amips.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/orient.hpp>

#include <wmtk/utils/Logger.hpp>

namespace wmtk::invariants {

CollapseEnergyBeforeInvariant::CollapseEnergyBeforeInvariant(
    const Mesh& m,
    const attribute::TypedAttributeHandle<Rational>& coordinate,
    const attribute::TypedAttributeHandle<double>& energy,
    int64_t collapse_type)
    : Invariant(m, true, false, false)
    , m_coordinate_handle(coordinate)
    , m_energy_handle(energy)
    , m_collapse_type(collapse_type)
{}

bool CollapseEnergyBeforeInvariant::before(const simplex::Simplex& s) const
{
    constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Triangle;
    constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;

    if (mesh().top_simplex_type() == PT) {
        auto position_accessor = mesh().create_const_accessor(m_coordinate_handle);

        auto amips_accessor = mesh().create_const_accessor(m_energy_handle);

        double old_energy_max = std::numeric_limits<double>::lowest();
        double new_energy_max = std::numeric_limits<double>::lowest();

        switch (m_collapse_type) {
        case 0: {
            const Tuple v0 = s.tuple();
            const Tuple v1 = mesh().switch_tuple(v0, PV);
            const simplex::Simplex v0_s(mesh(), PV, v0);
            const simplex::Simplex v1_s(mesh(), PV, v1);
            const auto v1_pos = position_accessor.const_vector_attribute(v1);

            // TODO: check sort and clean
            const auto& v0_incident_tets =
                simplex::top_dimension_cofaces(mesh(), simplex::Simplex::vertex(mesh(), v0), false);
            const auto& v1_incident_tets =
                simplex::top_dimension_cofaces(mesh(), simplex::Simplex::vertex(mesh(), v1), false);

            // compute energy for v1 incident tets
            for (const auto& t : v1_incident_tets) {
                auto lv = mesh().orient_vertices(t.tuple()); // get orient tet vertices
                assert(lv.size() == 4);

                // compute old max
                old_energy_max =
                    std::max(old_energy_max, amips_accessor.const_scalar_attribute(t.tuple()));

                // skip if incident both vertices
                bool incident_both = false;
                for (int i = 0; i < 4; ++i) {
                    if (simplex::utils::SimplexComparisons::equal(
                            mesh(),
                            simplex::Simplex::vertex(mesh(), lv[i]),
                            v0_s)) {
                        incident_both = true;
                        break;
                    }
                }
                if (incident_both) continue;

                // position after the collapse
                std::array<Eigen::Vector<Rational, 3>, 4> lv_pos = {
                    {position_accessor.const_vector_attribute(lv[0]),
                     position_accessor.const_vector_attribute(lv[1]),
                     position_accessor.const_vector_attribute(lv[2]),
                     position_accessor.const_vector_attribute(lv[3])}};

                for (int i = 0; i < 4; ++i) {
                    if (simplex::utils::SimplexComparisons::equal(
                            mesh(),
                            simplex::Simplex::vertex(mesh(), lv[i]),
                            v1_s)) {
                        // change the v1 entry to v0
                        lv_pos[i] = position_accessor.const_vector_attribute(v0);
                        break;
                    }
                    assert(i != 3); // should find one and break;
                }

                // check inversion, any new tet inverted return false
                if (wmtk::utils::wmtk_orient3d(lv_pos[0], lv_pos[1], lv_pos[2], lv_pos[3]) <= 0) {
                    return false;
                }

                double new_energy = wmtk::function::utils::Tet_AMIPS_energy(
                    {{lv_pos[0][0].to_double(),
                      lv_pos[0][1].to_double(),
                      lv_pos[0][2].to_double(),
                      lv_pos[1][0].to_double(),
                      lv_pos[1][1].to_double(),
                      lv_pos[1][2].to_double(),
                      lv_pos[2][0].to_double(),
                      lv_pos[2][1].to_double(),
                      lv_pos[2][2].to_double(),
                      lv_pos[3][0].to_double(),
                      lv_pos[3][1].to_double(),
                      lv_pos[3][2].to_double()}});

                new_energy_max = std::max(new_energy_max, new_energy);
            }

            // passed all inversion check, return true if v1 is not rounded
            for (int64_t i = 0; i < 3; ++i) {
                // if the collapse from vertex is not rounded, return true anyway
                if (!v1_pos[i].is_rounded()) return true;
            }

            // compute energy for v0 incident, old and new are the same because v0 is not moved
            for (const auto& t : v0_incident_tets) {
                auto lv = mesh().orient_vertices(t.tuple()); // get orient tet vertices

                // skip if incident both vertices
                bool incident_both = false;
                for (int i = 0; i < 4; ++i) {
                    if (simplex::utils::SimplexComparisons::equal(
                            mesh(),
                            simplex::Simplex::vertex(mesh(), lv[i]),
                            v1_s)) {
                        incident_both = true;
                        break;
                    }
                }
                if (incident_both) continue;

                // compute old max
                const double energy = amips_accessor.const_scalar_attribute(t.tuple());
                old_energy_max = std::max(old_energy_max, energy);

                // compute new energy
                new_energy_max = std::max(new_energy_max, energy);
            }

            break;
        }
        case 1: {
            const Tuple v0 = s.tuple();
            const Tuple v1 = mesh().switch_tuple(v0, PV);
            const simplex::Simplex v0_s(mesh(), PV, v0);
            const simplex::Simplex v1_s(mesh(), PV, v1);
            const auto v0_pos = position_accessor.const_vector_attribute(v0);

            // TODO: check sort and clean
            const auto& v0_incident_tets =
                simplex::top_dimension_cofaces(mesh(), simplex::Simplex::vertex(mesh(), v0), false);
            const auto& v1_incident_tets =
                simplex::top_dimension_cofaces(mesh(), simplex::Simplex::vertex(mesh(), v1), false);

            // compute energy for v0 incident tets
            for (const auto& t : v0_incident_tets) {
                auto lv = mesh().orient_vertices(t.tuple()); // get orient tet vertices
                assert(lv.size() == 4);

                // compute old max
                old_energy_max =
                    std::max(old_energy_max, amips_accessor.const_scalar_attribute(t.tuple()));

                // compute new energy
                // skip if incident both vertices
                bool incident_both = false;
                for (int i = 0; i < 4; ++i) {
                    if (simplex::utils::SimplexComparisons::equal(
                            mesh(),
                            simplex::Simplex::vertex(mesh(), lv[i]),
                            v1_s)) {
                        incident_both = true;
                        break;
                    }
                }
                if (incident_both) continue;

                // position after the collapse
                std::array<Eigen::Vector<Rational, 3>, 4> lv_pos = {
                    {position_accessor.const_vector_attribute(lv[0]),
                     position_accessor.const_vector_attribute(lv[1]),
                     position_accessor.const_vector_attribute(lv[2]),
                     position_accessor.const_vector_attribute(lv[3])}};

                for (int i = 0; i < 4; ++i) {
                    if (simplex::utils::SimplexComparisons::equal(
                            mesh(),
                            simplex::Simplex::vertex(mesh(), lv[i]),
                            v0_s)) {
                        // change the v0 entry to v1
                        lv_pos[i] = position_accessor.const_vector_attribute(v1);
                        break;
                    }
                    assert(i != 3); // should find one and break;
                }

                // check inversion, any new tet inverted return false
                if (wmtk::utils::wmtk_orient3d(lv_pos[0], lv_pos[1], lv_pos[2], lv_pos[3]) <= 0) {
                    return false;
                }

                double new_energy = wmtk::function::utils::Tet_AMIPS_energy(
                    {{lv_pos[0][0].to_double(),
                      lv_pos[0][1].to_double(),
                      lv_pos[0][2].to_double(),
                      lv_pos[1][0].to_double(),
                      lv_pos[1][1].to_double(),
                      lv_pos[1][2].to_double(),
                      lv_pos[2][0].to_double(),
                      lv_pos[2][1].to_double(),
                      lv_pos[2][2].to_double(),
                      lv_pos[3][0].to_double(),
                      lv_pos[3][1].to_double(),
                      lv_pos[3][2].to_double()}});
                new_energy_max = std::max(new_energy_max, new_energy);
            }

            // passed all inversion check, return true if v0 is not rounded
            for (int64_t i = 0; i < 3; ++i) {
                // if the collapse from vertex is not rounded, return true anyway
                if (!v0_pos[i].is_rounded()) return true;
            }

            // compute energy for v1 incident, old and new are the same because v0 is not moved
            for (const auto& t : v1_incident_tets) {
                auto lv = mesh().orient_vertices(t.tuple()); // get orient tet vertices

                // skip if incident both vertices
                bool incident_both = false;
                for (int i = 0; i < 4; ++i) {
                    if (simplex::utils::SimplexComparisons::equal(
                            mesh(),
                            simplex::Simplex::vertex(mesh(), lv[i]),
                            v0_s)) {
                        incident_both = true;
                        break;
                    }
                }
                if (incident_both) continue;

                // compute old max
                const double energy = amips_accessor.const_scalar_attribute(t.tuple());
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
