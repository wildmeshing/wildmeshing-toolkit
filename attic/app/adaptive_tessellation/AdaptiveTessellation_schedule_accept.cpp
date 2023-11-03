#include "AdaptiveTessellation.h"


namespace adaptive_tessellation {

double AdaptiveTessellation::get_2d_tri_area(const Tuple& t) const
{
    const auto& p1 = get_vertex_attrs(t).pos;
    const auto& p2 = get_vertex_attrs(t.switch_vertex(*this)).pos;
    const auto& p3 = get_vertex_attrs(t.switch_edge(*this).switch_vertex(*this)).pos;
    return wmtk::triangle_2d_area<double>(p1, p2, p3);
}

double AdaptiveTessellation::get_3d_tri_area(const Tuple& t) const
{
    const auto& p1_2d = get_vertex_attrs(t).pos;
    const auto& p2_2d = get_vertex_attrs(t.switch_vertex(*this)).pos;
    const auto& p3_2d = get_vertex_attrs(t.switch_edge(*this).switch_vertex(*this)).pos;
    const auto& p1 = mesh_parameters.m_displacement->get(p1_2d(0), p1_2d(1));
    const auto& p2 = mesh_parameters.m_displacement->get(p2_2d(0), p2_2d(1));
    const auto& p3 = mesh_parameters.m_displacement->get(p3_2d(0), p3_2d(1));

    return wmtk::triangle_3d_area<double>(p1, p2, p3);
}

double AdaptiveTessellation::get_max_energy_area_ratio(const std::vector<Tuple>& new_tris) const
{
    double max_energy_area_ratio = 0;
    for (const Tuple& t : new_tris) {
        double area_3d = get_3d_tri_area(t);
        if (area_3d < std::numeric_limits<double>::denorm_min()) {
            return std::numeric_limits<double>::infinity();
        }
        double energy_area_ratio =
            get_face_attrs(t).accuracy_measure.cached_distance_integral / area_3d;

        if (energy_area_ratio > max_energy_area_ratio) {
            max_energy_area_ratio = energy_area_ratio;
        }
    }
    return max_energy_area_ratio;
}
// accept split if max per face energy of the new tris is greater than the acceptance bound
bool AdaptiveTessellation::scheduling_accept_for_split(
    const std::vector<Tuple>& new_tris,
    double acceptance) const
{
    double max_energy_area_ratio = get_max_energy_area_ratio(new_tris);
    return sqrt(max_energy_area_ratio) > acceptance;
}
// accept collapse if max per face energy of the new tris is smaller than the acceptance bound
bool AdaptiveTessellation::scheduling_accept_for_collapse(
    const std::vector<Tuple>& new_tris,
    double acceptance) const
{
    double max_energy_area_ratio = get_max_energy_area_ratio(new_tris);
    return sqrt(max_energy_area_ratio) < acceptance;
}
// accept swap if max per face energy of the new tris is smaller than the acceptance bound
// and Daniel's other condition?
bool AdaptiveTessellation::scheduling_accept_for_swap(
    const std::vector<Tuple>& new_tris,
    double acceptance) const
{
    double max_energy_area_ratio = get_max_energy_area_ratio(new_tris);
    return sqrt(max_energy_area_ratio) < acceptance;
}
} // namespace adaptive_tessellation