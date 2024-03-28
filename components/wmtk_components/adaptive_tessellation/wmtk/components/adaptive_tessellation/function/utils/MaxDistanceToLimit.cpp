#include "MaxDistanceToLimit.hpp"

namespace wmtk::components::function::utils {
MaxDistanceToLimit::MaxDistanceToLimit(const ThreeChannelPositionMapEvaluator& evaluator)
    : Triangle2DTo3DMapping(evaluator)
{}

double MaxDistanceToLimit::distance(
    const Vector2<double>& uv0,
    const Vector2<double>& uv1,
    const Vector2<double>& uv2) const
{
    return 0;
}
} // namespace wmtk::components::function::utils