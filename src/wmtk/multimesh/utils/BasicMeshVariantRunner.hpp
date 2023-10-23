#pragma once

#include <wmtk/utils/metaprogramming/as_mesh_variant.hpp>
#include "CachedMeshVariantReturnValues.hpp"
namespace wmtk::multimesh::utils {
template <typename Functor, typename... OtherTypes>
class BasicMeshVariantRunner
{
public:
    BasicMeshVariantRunner(Functor&& f)
        : func(f)
    {}
    BasicMeshVariantRunner(Functor&& f, std::tuple<OtherTypes...>)
        : func(f)
    {}

    void run(Mesh& mesh, const OtherTypes&... ts)
    {
        auto var = wmtk::utils::metaprogramming::as_mesh_variant(mesh);
        std::visit(
            [&](auto& t) {
                auto& v = t.get();
                return_data.add(v, func(v, ts...), ts...);
            },
            var);
    }
    void run(const Mesh& mesh, const OtherTypes&... ts)
    {
        auto var = wmtk::utils::metaprogramming::as_const_mesh_variant(mesh);
        std::visit(
            [&](auto& t) {
                const auto& v = t.get();
                return_data.add(v, func(v, ts...), ts...);
            },
            var);
    }

    CachedMeshVariantReturnValues<Functor, OtherTypes...> return_data;

private:
    const Functor& func;
};

template <typename Functor, typename... Ts>
BasicMeshVariantRunner(Functor&& f, std::tuple<Ts...>)
    -> BasicMeshVariantRunner<Functor, std::decay_t<Ts>...>;
template <typename Functor>
BasicMeshVariantRunner(Functor&& f) -> BasicMeshVariantRunner<Functor>;
} // namespace wmtk::multimesh::utils
