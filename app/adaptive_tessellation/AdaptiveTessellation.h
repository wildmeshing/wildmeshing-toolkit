#pragma once

#include <igl/Timer.h>
#include <igl/doublearea.h>
#include <lagrange/SurfaceMesh.h>
#include <lagrange/bvh/EdgeAABBTree.h>
#include <wmtk/TriMesh.h>
#include <wmtk/utils/BoundaryParametrization.h>
#include <wmtk/utils/Displacement.h>
#include <wmtk/utils/DisplacementBicubic.h>
#include <wmtk/utils/Energy2d.h>
#include <wmtk/utils/GeoUtils.h>
#include <wmtk/utils/Image.h>
#include <wmtk/utils/MipMap.h>
#include <finitediff.hpp>
#include <nlohmann/json.hpp>
#include <sec/envelope/SampleEnvelope.hpp>
#include <wmtk/utils/LineQuadrature.hpp>
#include "Parameters.h"

namespace adaptive_tessellation {
class VertexAttributes
{
public:
    Eigen::Vector2d pos;
    double t = 0.;
    size_t curve_id = 0; // questionable should I have this for each vertex?

    size_t partition_id = 0; // TODO this should not be here

    // Vertices marked as fixed cannot be modified by any local operation
    bool fixed = false;
    bool boundary_vertex = false;
};

class AdaptiveTessellation : public wmtk::TriMesh
{
public:
    template <class T>
    using RowMatrix2 = Eigen::Matrix<T, Eigen::Dynamic, 2, Eigen::RowMajor>;
    using Index = uint64_t;
    using Scalar = double;
    using DScalar = DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d>;

public:
    Parameters mesh_parameters;
    // Store the per-vertex attributes
    wmtk::AttributeCollection<VertexAttributes> vertex_attrs;
    struct InfoCache
    {
        size_t v1;
        size_t v2;
        double max_energy;
        int partition_id;
    };
    tbb::enumerable_thread_specific<InfoCache> cache;

public:
    AdaptiveTessellation(){};

    virtual ~AdaptiveTessellation(){};

    void set_output_folder(std::filesystem::path output_folder)
    {
        mesh_parameters.m_output_folder = output_folder.string();
    }
    // set often used parameters in a bundle. User can also set each used parameters that are
    // used separately
    void set_parameters(
        const double target_edge_length,
        const wmtk::Image& image,
        const WrappingMode wrapping_mode,
        const EDGE_LEN_TYPE edge_len_type,
        const ENERGY_TYPE energy_type,
        const bool boundary_parameter);
    void set_parameters(
        const double target_edge_length,
        const std::function<DScalar(const DScalar&, const DScalar&)>& displacement_function,
        const EDGE_LEN_TYPE edge_len_type,
        const ENERGY_TYPE energy_type,
        const bool boundary_parameter);

    void set_energy(const ENERGY_TYPE energy_type);
    void set_energy(std::unique_ptr<wmtk::Energy> f) { mesh_parameters.m_energy = std::move(f); };
    void set_image_function(const wmtk::Image& image, const WrappingMode wrapping_mode);
    void set_displacement();
    void set_edge_length_measurement(const EDGE_LEN_TYPE edge_len_type);
    void set_projection();
    Eigen::Matrix<uint64_t, Eigen::Dynamic, 2, Eigen::RowMajor> get_bnd_edge_matrix();


    bool invariants(const std::vector<Tuple>& new_tris);

    void set_feature(Tuple& t); // find the feature vertex and freeze them

    // Initializes the mesh
    /**
     * @brief Create a mesh object. Assumed the parameters are already set before here. Can,
     * therefore, construct member variables according to the parameters
     *
     * @param V igl format vertices
     * @param F igl format faces
     */
    void create_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);

    // Exports V and F of the stored mesh
    void export_mesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F);

    // Writes a triangle mesh in OBJ format
    void write_obj(const std::string& path);

    // Writes a triangle mesh in ply format
    void write_ply(const std::string& path);
    void write_vtk(const std::string& path);

    void write_displaced_obj(
        const std::string& path,
        const std::function<double(double, double)>& displacement);
    void write_displaced_obj(
        const std::string& path,
        const std::function<Eigen::Vector3d(double, double)>& displacement);

    // Computes the quality of a triangle
    double get_quality(const Tuple& loc, int idx = 0) const;
    std::pair<double, Eigen::Vector2d> get_one_ring_energy(const Tuple& loc) const;

    // Computes the average quality of a mesh
    Eigen::VectorXd get_quality_all_triangles();

    // Check if a triangle is inverted
    bool is_inverted(const Tuple& loc) const;

    std::vector<TriMesh::Tuple> new_edges_after(const std::vector<TriMesh::Tuple>& tris) const;

    // Smoothing
    void smooth_all_vertices();
    bool smooth_before(const Tuple& t) ;
    bool smooth_after(const Tuple& t) ;

    // Collapse
    void collapse_all_edges();
    bool collapse_edge_before(const Tuple& t) ;
    bool collapse_edge_after(const Tuple& t) ;
    // Split
    void split_all_edges();
    bool split_edge_before(const Tuple& t) ;
    bool split_edge_after(const Tuple& t) ;
    // Swap
    void swap_all_edges();
    bool swap_edge_before(const Tuple& t) ;
    bool swap_edge_after(const Tuple& t) ;

    void mesh_improvement(int max_its);
    void gradient_debug(int max_its);

    double get_length2d(const size_t& vid1, const size_t& vid2) const;
    double get_length3d(const size_t& vid1, const size_t& vid2)
        const; // overload of the version that takes a tuple.
               // used when the tuple is invalid but use vids to uquest for positions in the
               // vertex_attrs
    double get_length_n_implicit_points(const size_t& vid1, const size_t& vid2) const;
    double get_length_1ptperpixel(const size_t& vid1, const size_t& vid2) const;
    double get_length_mipmap(const size_t& vid1, const size_t& vid2) const;

    void flatten_dofs(Eigen::VectorXd& v_flat);
    double get_mesh_energy(const Eigen::VectorXd& v_flat);

    double get_accuracy_error(const size_t& vid1, const size_t& vid2) const;
    double get_accuracy_error(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2)
        const; // outdated

    template <class T, int order>
    inline std::decay_t<T> quadrature_error_1pixel_eval(
        const Eigen::Matrix<T, 2, 3> edge_verts,
        std::function<T(const T&, const T&)> image_get_z,
        wmtk::LineQuadrature& quad) const
    {
        quad.get_quadrature(order);
        double ret = 0.0;
        auto v1z = edge_verts(0, 2);
        auto v2z = edge_verts(1, 2);
        Eigen::Matrix<T, 1, 2> v12d, v22d;
        v12d << edge_verts(0, 0), edge_verts(0, 1);
        v22d << edge_verts(1, 0), edge_verts(1, 1);
        // now do 1d quadrature
        for (int i = 0; i < quad.points.rows(); i++) {
            auto tmpu =
                (1 - quad.points(i, 0)) * edge_verts(0, 0) + quad.points(i, 0) * edge_verts(1, 0);
            auto tmpv =
                (1 - quad.points(i, 0)) * edge_verts(0, 1) + quad.points(i, 0) * edge_verts(1, 1);
            auto tmph = image_get_z(tmpu, tmpv);
            auto tmpz = (1 - quad.points(i, 0)) * v1z + quad.points(i, 0) * v2z;
            ret += abs(quad.weights(i) * (tmph - tmpz));
        }

        return ret * (v12d - v22d).stableNorm();
    }
};

} // namespace adaptive_tessellation
