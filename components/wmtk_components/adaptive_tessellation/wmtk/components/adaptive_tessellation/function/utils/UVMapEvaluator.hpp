#pragma once
#include <memory>
#include <wmtk/Types.hpp>
namespace wmtk::components {
namespace function::utils {
class UVMapEvaluator
{
public:
    UVMapEvaluator();
    ~UVMapEvaluator();

    template <typename T>
    Vector2<T> t_to_uv(
        const T& t,
        const T& tl,
        const T& tr,
        const Vector2<T>& uvl,
        const Vector2<T>& uvr) const
    {
        // assuming t is in [tl, tr]
        if (tr < tl) {
            assert(t >= tr && t <= tl);
        } else {
            assert(t >= tl && t <= tr);
        }
        return uvl + (uvr - uvl) * (t - tl) / (tr - tl);
    }
};
} // namespace function::utils
} // namespace wmtk::components