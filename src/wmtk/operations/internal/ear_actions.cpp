#include "ear_actions.hpp"
#include <wmtk/autogen/SimplexDart.hpp>
namespace wmtk::operations::internal {
namespace {
auto make_opp_actions() -> std::array<int8_t, 4>
{
    std::array<int8_t, 4> darts;
    darts[0] = 0;
    for (int8_t j = 1; j < darts.size(); ++j) {
        PrimitiveType pt = get_primitive_type_from_id(j);
        wmtk::autogen::SimplexDart sd(pt);
        darts[j] = sd.opposite();
    }
    //
    return darts;
}
auto make_right_ear_darts() -> std::array<int8_t, 4>
{
    return make_opp_actions();
}
auto make_left_ear_darts() -> std::array<int8_t, 4>
{
    auto darts = make_opp_actions();
    for (int8_t j = 1; j < darts.size(); ++j) {
        PrimitiveType pt = get_primitive_type_from_id(j);
        wmtk::autogen::SimplexDart sd(pt);
        int8_t& action = darts[j];
        action = sd.product(action, sd.primitive_as_index(wmtk::PrimitiveType::Vertex));
    }
    return darts;
}
const static std::array<int8_t, 4> left_ear_darts = make_left_ear_darts();
const static std::array<int8_t, 4> right_ear_darts = make_right_ear_darts();

} // namespace
int8_t left_ear_action(PrimitiveType mesh_type)
{
    const size_t off = get_primitive_type_id(mesh_type);
    return left_ear_darts[off];
}
int8_t right_ear_action(PrimitiveType mesh_type)
{
    const size_t off = get_primitive_type_id(mesh_type);
    return right_ear_darts[off];
}

int8_t ear_action(PrimitiveType mesh_dimension, bool is_left) {
    if(is_left) {
        return left_ear_action(mesh_dimension);
    } else {
        return right_ear_action(mesh_dimension);
    }
}
std::array<int8_t,2> ear_actions(PrimitiveType mesh_dimension) {
    return std::array<int8_t,2> {{left_ear_action(mesh_dimension),right_ear_action(mesh_dimension)}};
}
} // namespace wmtk::operations::internal
