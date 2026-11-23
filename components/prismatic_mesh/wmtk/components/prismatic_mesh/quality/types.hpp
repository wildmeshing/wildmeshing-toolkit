#pragma once
#include <chrono>
#include <functional>
#include <queue>
#include "native_refinement.hpp"
namespace wmtk::components::prismatic_mesh::quality {
using Json = nlohmann::json;
using Tet = std::array<size_t, 4>;
using native::Mesh;
using native::V;
struct Deadline
{
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    double limit;
    explicit Deadline(double seconds)
        : limit(seconds)
    {}
    double seconds() const
    {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    }
    bool expired() const { return seconds() >= limit; }
};
inline size_t append_volume_vertex(Mesh& mesh, const V& point, double sizing = 1)
{
    const size_t v = mesh.p.size();
    mesh.p.push_back(point);
    mesh.initial.push_back(point);
    mesh.base.push_back(point);
    mesh.thickness.push_back(std::numeric_limits<double>::quiet_NaN());
    mesh.radius.push_back(0);
    mesh.sizing.push_back(sizing);
    mesh.input.push_back(false);
    mesh.outer.push_back(false);
    mesh.correspondence.push_back(-1);
    return v;
}
} // namespace wmtk::components::prismatic_mesh::quality
