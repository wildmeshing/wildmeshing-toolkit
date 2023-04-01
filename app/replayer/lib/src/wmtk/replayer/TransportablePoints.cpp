#include <tbb/parallel_for.h>
#include <wmtk/utils/TransportablePoints.hpp>


using namespace wmtk;


TransportablePointsBase::~TransportablePointsBase() = default;

void TransportablePointsBase::before_hook(
    const TriMesh& m,
    const std::vector<TriMesh::Tuple>& input_tris)
{
    std::vector<size_t> tids;
    std::transform(
        std::begin(input_tris),
        std::end(input_tris),
        std::back_inserter(tids),
        [&](const auto& t) { return t.fid(m); });
    spdlog::info("TransportablePointsBase::before_hook had tris {}", fmt::join(tids, ","));
    std::set<size_t> input_tri_set;
    std::transform(
        input_tris.begin(),
        input_tris.end(),
        std::inserter(input_tri_set, input_tri_set.end()),
        [&](const TriMesh::Tuple& t) -> size_t { return t.fid(m); });

    std::set<size_t>& active_pts = active_points.local();
    // go through set of input tris and try to move every point in an output tri
    for (size_t point_index = 0; point_index < triangle_tuples.size(); ++point_index) {
        const TriMesh::Tuple& tup = triangle_tuples[point_index];
        size_t fid = tup.fid(m);
        if (input_tri_set.find(fid) != input_tri_set.end()) {
            active_pts.emplace(point_index);
            update_global_coordinate(m, point_index);
        }
    }
    spdlog::info("TransportablePointsBase::before_hook done");
}

TriMesh::Tuple TransportablePointsBase::get_point_triangle(
    const TriMesh& m,
    const std::vector<TriMesh::Tuple>& triangles,
    size_t point_index) const
{
    for (const auto& t : triangles) {
        if (point_in_triangle(m, t, point_index)) {
            spdlog::info("Found point {} in triangle {}", point_index, t.fid(m));
            return t;
        }
    }
    spdlog::error("was unable to find point {}", point_index);
    assert(false); // get_point_triangle did not find a triangle

    return {};
}
void TransportablePointsBase::update_local_coordinates(const TriMesh& m)
{

    update_triangle_tuples(m);
    update_barycentric_coordinates(m);
}

void TransportablePointsBase::update_triangle_tuples(const TriMesh& m)
{
    triangle_tuples.resize(size());
    std::vector<TriMesh::Tuple> tuples = m.get_faces();

    tbb::parallel_for(size_t(0), size(), [&](size_t j) {
        triangle_tuples[j] = get_point_triangle(m, tuples, j);
    });
}

void TransportablePointsBase::update_barycentric_coordinates(const TriMesh& m)
{
    barycentric_coordinates.resize(size());

    tbb::parallel_for(size_t(0), size(), [&](size_t j) {
        barycentric_coordinates[j] = get_barycentric(m, triangle_tuples.at(j), j);
    });
}

void TransportablePointsBase::after_hook(
    const TriMesh& m,
    const std::vector<TriMesh::Tuple>& output_tris)
{
    std::vector<size_t> tids;
    std::transform(
        std::begin(output_tris),
        std::end(output_tris),
        std::back_inserter(tids),
        [&](const auto& t) { return t.fid(m); });
    spdlog::info("TransportablePointsBase::after_hook had tris {} and active points {}", fmt::join(tids, ","), fmt::join(active_points.local(), ","));
    // go through set of input tris and try to move every point in an output tri
    for (const size_t point_index : active_points.local()) {
        spdlog::info("Updating {} / {}", point_index, size());
        update_local_coordinate(m, point_index, output_tris);
    }
    spdlog::info("Done updating local coordinates");
    active_points.local().clear();
}


// derived class is required to store a global representation of the point, used in before_hook
void TransportablePointsBase::update_local_coordinate(
    const TriMesh& m,
    size_t point_index,
    const std::vector<TriMesh::Tuple>& possible_tris)
{
    spdlog::info("updating local coordinate {}", point_index);
    for(const auto& t: possible_tris) {
        spdlog::info("{}", fmt::join(m.oriented_tri_vids(t),","));
    }
    // this point needs to be moved forward in this operation
    // try to see if it's in a triangle
    bool found = false;
    for (const TriMesh::Tuple& tup : possible_tris) {
        // const size_t triangle_index = tup.fid(m);
        if (point_in_triangle(m, tup, point_index)) {
            triangle_tuples[point_index] = tup;
            const auto& bary = barycentric_coordinates[point_index] = get_barycentric(m, tup, point_index);
            spdlog::info("Point {} tup is {} and bary is {}", point_index, tup.info(), bary);
            found = true;
        }
    }

    if (!found) {
        spdlog::warn("Point not found in a triangle, backup mechanism required");
    }
}
