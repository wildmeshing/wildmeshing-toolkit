#pragma once
#include <memory>
#include <wmtk/Types.hpp>

namespace wmtk::function::utils {
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
        return uvl + (uvr - uvl) * (t - tl) / (tr - tl);
    }
};
} // namespace wmtk::function::utils