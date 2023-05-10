#pragma once

#include <fastenvelope/FastEnvelope.h>
#include <igl/Timer.h>
#include <igl/doublearea.h>
#include <igl/predicates/predicates.h>
#include <igl/read_triangle_mesh.h>
#include <igl/writeDMAT.h>
#include <igl/write_triangle_mesh.h>
#include <lagrange/SurfaceMesh.h>
#include <lagrange/attribute_names.h>
#include <lagrange/bvh/EdgeAABBTree.h>
#include <lagrange/foreach_attribute.h>
#include <lagrange/io/load_mesh.h>
#include <lagrange/triangulate_polygonal_facets.h>
#include <lagrange/utils/fpe.h>
#include <lagrange/utils/timing.h>
#include <lagrange/views.h>
#include <tbb/concurrent_vector.h>
#include <wmtk/TriMesh.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/AMIPS2D_autodiff.h>
#include <wmtk/utils/BoundaryParametrization.h>
#include <wmtk/utils/Displacement.h>
#include <wmtk/utils/Energy2d.h>
#include <wmtk/utils/Energy2dOptimizationUtils.h>
#include <wmtk/utils/GeoUtils.h>
#include <wmtk/utils/Image.h>
#include <wmtk/utils/MipMap.h>
#include <wmtk/utils/PolygonClipping.h>
#include <Eigen/Core>
#include <finitediff.hpp>
#include <lean_vtk.hpp>
#include <nlohmann/json.hpp>
#include <sec/envelope/SampleEnvelope.hpp>
#include <wmtk/utils/LineQuadrature.hpp>
#include <wmtk/utils/TriQualityUtils.hpp>
#include <wmtk/utils/TupleUtils.hpp>
#include "Parameters.h"

namespace adaptive_tessellation {
class VertexAttributes
{
public:
    Eigen::Vector2d pos;
    double t = 0.;
    size_t curve_id = 0; // TODO questionable should I have this for each vertex? change to be for
                         // each edge, but still keep one copy for vertex

    size_t partition_id = 0; // TODO this should not be here

    // Vertices marked as fixed cannot be modified by any local operation
    bool fixed = false;
    bool boundary_vertex = false;
};

class FaceAttributes
{
public:
    std::array<std::optional<wmtk::TriMesh::Tuple>, 3> mirror_edges;
};

class EdgeAttributes
{
public:
    std::optional<int> curve_id = std::nullopt;
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
    wmtk::AttributeCollection<FaceAttributes> face_attrs;
    wmtk::AttributeCollection<EdgeAttributes> edge_attrs;
    struct InfoCache
    {
        size_t v1;
        size_t v2;
        double error;
        double max_energy;
        int partition_id;
    };
    tbb::enumerable_thread_specific<InfoCache> cache;

    //////// ======= seam vertex coloring ========
    // both coloring mappings contains the regular seam vertex and v shape seams vertex
    // mapping is built at loading and is not maintained during the mesh operations
    // since seam vertices at t-junctions should not be modified,
    // those colorings do not need updates
    std::unordered_map<size_t, int> uv_index_to_color;
    // each color can have 1 vertex, 2 vertices, or 3 above vertices
    std::vector<std::vector<size_t>> color_to_uv_indices;

public:
    AdaptiveTessellation(){};

    virtual ~AdaptiveTessellation(){};

    void create_paired_seam_mesh_with_offset(
        const std::filesystem::path input_mesh_path,
        Eigen::MatrixXd& UV,
        Eigen::MatrixXi& F);

    void set_output_folder(std::filesystem::path output_folder)
    {
        mesh_parameters.m_output_folder = output_folder.string();
    }
    // set often used parameters in a bundle. User can also set each used parameters that are
    // used separately
    void set_parameters(
        const double target_accuracy,
        const double target_edge_length,
        const wmtk::Image& image,
        const WrappingMode wrapping_mode,
        const SAMPLING_MODE sampling_mode,
        const DISPLACEMENT_MODE displacement_mode,
        const ENERGY_TYPE energy_type,
        const EDGE_LEN_TYPE edge_len_type,
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
    void set_displacement(const DISPLACEMENT_MODE displacement_mode);
    void set_edge_length_measurement(const EDGE_LEN_TYPE edge_len_type);
    void set_projection();
    // using boundary parametrization, find the vertex that are the start and end of each cruve and
    // set them as fixed
    void set_fixed();
    void assign_edge_curveid();
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
    void mesh_construct_boundaries(
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXi& F,
        const Eigen::MatrixXi& E0,
        const Eigen::MatrixXi& E1);

    // Exports V and F of the stored mesh
    void export_mesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F) const;

    // Exports V and F of the stored mesh where all seam vertices are merged
    void remove_seams(Eigen::MatrixXd& V, Eigen::MatrixXi& F) const;

    // Writes a triangle mesh in OBJ format
    void write_obj(const std::string& path);

    // Writes a triangle mesh in ply format
    void write_ply(const std::string& path);
    void write_vtk(const std::string& path);
    void write_perface_vtk(const std::string& path);

    void write_displaced_obj(
        const std::string& path,
        const std::function<double(double, double)>& displacement);
    void write_displaced_obj(
        const std::string& path,
        const std::shared_ptr<wmtk::Displacement> displacement);
    void write_displaced_seamless_obj(
        const std::string& path,
        const std::shared_ptr<wmtk::Displacement> displacement);

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
    bool smooth_before(const Tuple& t);
    bool smooth_after(const Tuple& t);

    // Collapse
    void collapse_all_edges();
    bool collapse_edge_before(const Tuple& t);
    bool collapse_edge_after(const Tuple& t);
    // Split
    void split_all_edges();
    bool split_edge_before(const Tuple& t);
    bool split_edge_after(const Tuple& t);
    // Swap
    void swap_all_edges();
    bool swap_edge_before(const Tuple& t);
    bool swap_edge_after(const Tuple& t);

    void mesh_improvement(int max_its);
    void gradient_debug(int max_its);

    double get_length2d(const Tuple& edge_tuple) const;
    double get_length3d(
        const Tuple& edge_tuple) const; // overload of the version that takes a tuple.
                                        // used when the tuple is invalid but use vids to uquest for
                                        // positions in the vertex_attrs
    double get_length_n_implicit_points(const Tuple& edge_tuple) const;
    double get_length_1ptperpixel(const Tuple& edge_tuple) const;
    double get_length_mipmap(const Tuple& edge_tuple) const;

    void flatten_dofs(Eigen::VectorXd& v_flat);
    double get_mesh_energy(const Eigen::VectorXd& v_flat);

    double get_edge_accuracy_error(const Tuple& edge_tuple) const;
    double get_area_accuracy_error_per_face(const Tuple& edge_tuple) const;
    double get_area_accuracy_error_per_face_triangle_matrix(
        Eigen::Matrix<double, 3, 2, Eigen::RowMajor> triangle) const;
    double get_area_accuracy_error(const Tuple& edge_tuple) const;

    void get_nminfo_for_vertex(const Tuple& v, wmtk::NewtonMethodInfo& nminfo) const;

    // get sibling edge for paired operations
    // return the oriented mirror edge if t is seam
    // return the sibling if t is interior
    // return nullopt if t is boundary
    std::optional<TriMesh::Tuple> get_sibling_edge(const TriMesh::Tuple& t) const;
    // given a seam edge retrieve its mirror edge in opposite direction (half egde conventions )
    TriMesh::Tuple get_oriented_mirror_edge(const TriMesh::Tuple& t) const;
    // given a seam edge with vid v retrieve the correpsonding vertex on the mirror edge
    TriMesh::Tuple get_mirror_vertex(const TriMesh::Tuple& t) const;
    // return a vector of mirror vertices. store v itself at index 0 of the returned vector
    // !!! assume no operation has made fixed vertices outdated
    std::vector<TriMesh::Tuple> get_all_mirror_vertices(const TriMesh::Tuple& v);
    std::vector<size_t> get_all_mirror_vids(const TriMesh::Tuple& v);
    // set primary_t's mirror edge data to a ccw ordered mirror_edge
    void set_mirror_edge_data(const TriMesh::Tuple& primary_t, const TriMesh::Tuple& mirror_edge);
    bool is_seam_edge(const TriMesh::Tuple& t) const;
    bool is_seam_vertex(const TriMesh::Tuple& t) const;
    // unit test functions
    inline void create_mesh_debug(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
    {
        // Register attributes
        p_vertex_attrs = &vertex_attrs;
        p_face_attrs = &face_attrs;
        // Convert from eigen to internal representation (TODO: move to utils and remove it from all
        // app)
        std::vector<std::array<size_t, 3>> tri(F.rows());
        for (int i = 0; i < F.rows(); i++) {
            tri[i][0] = (size_t)F(i, 0);
            tri[i][1] = (size_t)F(i, 1);
            tri[i][2] = (size_t)F(i, 2);
        }
        // Initialize the trimesh class which handles connectivity
        wmtk::TriMesh::create_mesh(V.rows(), tri);
        // Save the vertex position in the vertex attributes
        for (unsigned i = 0; i < V.rows(); ++i) {
            vertex_attrs[i].pos << V.row(i)[0], V.row(i)[1];
        }
        // for (const auto& tri : this->get_faces()) {
        //     assert(!is_inverted(tri));
        // }
    }
};
} // namespace adaptive_tessellation
