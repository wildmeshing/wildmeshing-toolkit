#pragma once

#define USE_CALLBACK_FOR_TRANSPOORTABLE_POINTS

#include <wmtk/TriMesh.h>
#include <functional>


namespace wmtk {
class TransportablePointsBase
{
public:
    virtual ~TransportablePointsBase();
    // convenience function that just calls
    void before_hook(const TriMesh& m, const std::vector<TriMesh::Tuple>& input_tris);
    void after_hook(const TriMesh& m, const std::vector<TriMesh::Tuple>& output_tris);

    // derived class is required to store a global representation of the point, used in before_hook
    virtual void update_global_coordinate(const TriMesh& m, size_t point_index) = 0;

    void update_local_coordinates();
    virtual size_t size() const = 0;

    // call this to do a slow first initialization of points. better if the user specifies the
    // owning triangles on their own
    void update_local_coordinates(const TriMesh& m);
    // if the user only knows the owning triangles but not the barycentric coordinates they can call
    // this function, which assumes triangle_tuples is properly populated
    void update_barycentric_coordinates(const TriMesh& m);

protected:
    // does a O(NM) initialization for N triangles M points, meant to called from
    // "update_local_coordinates"
    void update_triangle_tuples(const TriMesh& m);
    // derived class is required to store a global representation of the point, used in before_hook
    void update_local_coordinate(
        const TriMesh& m,
        size_t point_index,
        const std::vector<TriMesh::Tuple>& possible_tris);
    // derived class is required to identify which point and triangle
    virtual bool point_in_triangle(const TriMesh& m, const TriMesh::Tuple& t, size_t point_index)
        const = 0;

    virtual std::array<double, 3>
    get_barycentric(const TriMesh& m, const TriMesh::Tuple& t, size_t point_index) const = 0;


    TriMesh::Tuple get_point_triangle(
        const TriMesh& m,
        const std::vector<TriMesh::Tuple>& triangles,
        size_t point_index) const;

    // Members section==================================
public:
    // local representation of points in a triangle mesh
    tbb::concurrent_vector<TriMesh::Tuple> triangle_tuples;
    tbb::concurrent_vector<std::array<double, 3>> barycentric_coordinates;

protected:
    tbb::enumerable_thread_specific<std::set<size_t>> active_points;

};

template <typename PointType>
class TransportablePoints : public TransportablePointsBase
{
public:
    using ThreePointType = const std::array<std::reference_wrapper<const PointType>, 3>;
    // derived class is required to store a global representation of the point, used in
    // before_hook
    void update_global_coordinate(const TriMesh& m, size_t point_index) override;

    // predicate to determine whether a point lies in a particular triangle
    bool point_in_triangle(const TriMesh& m, const TriMesh::Tuple& t, size_t point_index)
        const override;

    // computes the barycentric coordinates for the point at point_index assuming that it lies in
    // triangle_index
    std::array<double, 3>
    get_barycentric(const TriMesh& m, const TriMesh::Tuple& t, size_t point_index) const override;

    template <typename ContainerType>
    void set_points(const ContainerType& a)
    {
        points_global.resize(a.size());
        std::copy(std::begin(a), std::begin(a), std::begin(points_global));
        for(size_t j = 0; j < a.size(); ++j) {
            points_global[j] = a[j];
        }
        spdlog::info("Input Points Input:: {}", fmt::join(a,","));
        spdlog::info("Input Points Out:: {}", fmt::join(points_global,","));
    }
    size_t size() const override { return points_global.size(); }

protected:
    const AttributeCollection<PointType>& get_vertex_attributes(const TriMesh& m) const;
    ThreePointType get_points(const TriMesh& m, const TriMesh::Tuple& t) const;

#if defined(USE_CALLBACK_FOR_TRANSPOORTABLE_POINTS)

    // Mesh being used, barycentric coordinate in the desired triangle, and three point values in
    // attribute array
    using BarycentricInterpFuncType = std::function<
        PointType(const TriMesh&, const TriMesh::Tuple&, const std::array<double, 3>&)>;
    // mesh being used, barycentric coordinate in each desired triangle
    using PointInTriangleFuncType =
        std::function<bool(const TriMesh&, const TriMesh::Tuple&, const PointType&)>;
    //
    using GetBarycentricFuncType = std::function<
        std::array<double, 3>(const TriMesh&, const TriMesh::Tuple&, const PointType&)>;

public:
    BarycentricInterpFuncType barycentric_interp_callback;
    PointInTriangleFuncType point_in_triangle_callback;
    GetBarycentricFuncType get_barycentric_callback;
#endif
    // global coordinates
    tbb::concurrent_vector<PointType> points_global;
};


template <typename PointType>
auto TransportablePoints<PointType>::get_vertex_attributes(const TriMesh& m) const
    -> const AttributeCollection<PointType>&
{
    return dynamic_cast<const AttributeCollection<PointType>&>(*m.p_vertex_attrs);
}

template <typename PointType>
auto TransportablePoints<PointType>::get_points(const TriMesh& m, const TriMesh::Tuple& t) const
    -> ThreePointType
{
    const tbb::concurrent_vector<PointType>& P = get_vertex_attributes(m).m_attributes;
    const auto [ai, bi, ci] = m.oriented_tri_vids(t);
    const PointType& a = P[ai];
    const PointType& b = P[bi];
    const PointType& c = P[ci];
    return {a, b, c};
}

#if defined(USE_CALLBACK_FOR_TRANSPOORTABLE_POINTS)
template <typename PointType>
void TransportablePoints<PointType>::update_global_coordinate(const TriMesh& m, const size_t point_index)
{
    spdlog::info("update_global_coordinate({}) / {} {} {}", point_index,
            points_global.size(), triangle_tuples.size(), barycentric_coordinates.size()
            );
    const PointType old = points_global[point_index];
    const PointType& fresh = points_global[point_index] = barycentric_interp_callback(
        m,
        triangle_tuples[point_index],
        barycentric_coordinates[point_index]);

    spdlog::info("update_global_coordinate({}) {} => {}", point_index, old, fresh);
}

template <typename PointType>
bool TransportablePoints<PointType>::point_in_triangle(
    const TriMesh& m,
    const TriMesh::Tuple& t,
    size_t point_index) const
{
    //spdlog::info("point_in_triangle({})", point_index);
    return point_in_triangle_callback(m, t, points_global.at(point_index));
}

template <typename PointType>
std::array<double, 3> TransportablePoints<PointType>::get_barycentric(
    const TriMesh& m,
    const TriMesh::Tuple& t,
    size_t point_index) const
{
    //spdlog::info("get_barycentric({})", point_index);
    return get_barycentric_callback(m, t, points_global.at(point_index));
}

#endif


} // namespace wmtk
