#include "AdaptiveTessellation.h"
#include <fastenvelope/FastEnvelope.h>
#include <igl/Timer.h>
#include <igl/predicates/predicates.h>
#include <igl/writeDMAT.h>
#include <igl/write_triangle_mesh.h>
#include <lagrange/utils/timing.h>
#include <tbb/concurrent_vector.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/AMIPS2D_autodiff.h>
#include <Eigen/Core>
#include <lean_vtk.hpp>
#include <wmtk/utils/TriQualityUtils.hpp>
#include <wmtk/utils/TupleUtils.hpp>

using namespace wmtk;

namespace adaptive_tessellation {
auto avg_edge_len = [](auto& m) {
    double avg_len = 0.0;
    auto edges = m.get_edges();
    for (auto& e : edges)
        avg_len += std::sqrt(m.mesh_parameters.m_get_length(e.vid(m), e.switch_vertex(m).vid(m)));
    return avg_len / edges.size();
};

void AdaptiveTessellation::set_parameters(
    const double target_edge_length,
    const wmtk::Image& image,
    const WrappingMode wrapping_mode,
    const EDGE_LEN_TYPE edge_len_type,
    const ENERGY_TYPE energy_type,
    const bool boundary_parameter)
{
    switch (edge_len_type) {
    case EDGE_LEN_TYPE::ACCURACY: mesh_parameters.m_accuracy_threshold = target_edge_length; break;
    default: mesh_parameters.m_target_l = target_edge_length; break;
    }

    // setting needs to be in the order of image-> displacement-> energy-> edge_length
    // set the image first since it is used for displacement and energy setting
    set_image_function(image, wrapping_mode);
    set_displacement();
    set_energy(energy_type);
    set_edge_length_measurement(edge_len_type);
    mesh_parameters.m_boundary_parameter = boundary_parameter;
}

void AdaptiveTessellation::set_parameters(
    const double target_edge_length,
    const std::function<DScalar(const DScalar&, const DScalar&)>& displacement_function,
    const EDGE_LEN_TYPE edge_len_type,
    const ENERGY_TYPE energy_type,
    const bool boundary_parameter)
{
    mesh_parameters.m_target_l = target_edge_length;
    mesh_parameters.m_get_z = displacement_function;
    set_energy(
        energy_type); // set the displacement_function first since it is used for energy setting
    set_edge_length_measurement(edge_len_type); // set the default to get_legnth_3d
    mesh_parameters.m_boundary_parameter = boundary_parameter;
}

/// @brief set the v as feature vertex, that is, it is a boundary vertex and the  incident boundary vertices are not colinear
/// @param v
void AdaptiveTessellation::set_feature(Tuple& v)
{
    // only set the feature if it is a boundary vertex
    assert(is_boundary_vertex(v));
    // get 3 vertices

    std::vector<size_t> incident_boundary_verts;
    auto one_ring = get_one_ring_edges_for_vertex(v);
    for (auto e : one_ring) {
        if (is_boundary_edge(e)) incident_boundary_verts.emplace_back(e.vid(*this));
    }
    // every boundary vertex has to have 2 neighbors that are boundary vertices
    assert(incident_boundary_verts.size() == 2);
    auto p1 = vertex_attrs[v.vid(*this)].pos;
    auto p2 = vertex_attrs[incident_boundary_verts[0]].pos;
    auto p3 = vertex_attrs[incident_boundary_verts[1]].pos;
    // check if they are co linear. set fixed = true if not
    double costheta = ((p2 - p1).stableNormalized()).dot((p3 - p1).stableNormalized()); // cos theta
    double theta = std::acos(std::clamp<double>(costheta, -1, 1));
    if (theta <= M_PI / 2)
        vertex_attrs[v.vid(*this)].fixed = true;
    else
        vertex_attrs[v.vid(*this)].fixed = false;
}

// assuming the m_triwild_displacement in mesh_parameter has been set
void AdaptiveTessellation::set_energy(const ENERGY_TYPE energy_type)
{
    std::unique_ptr<Energy> energy_ptr;
    switch (energy_type) {
    case ENERGY_TYPE::AMIPS: energy_ptr = std::make_unique<wmtk::AMIPS>(); break;
    case ENERGY_TYPE::SYMDI: energy_ptr = std::make_unique<wmtk::SymDi>(); break;
    case ENERGY_TYPE::EDGE_LENGTH:
        energy_ptr = std::make_unique<wmtk::EdgeLengthEnergy>(mesh_parameters.m_get_z);
        break;
    case ENERGY_TYPE::EDGE_QUADRATURE:
        energy_ptr = std::make_unique<wmtk::AccuracyEnergy>(mesh_parameters.m_displacement);
    }

    mesh_parameters.m_energy = std::move(energy_ptr);
}

void AdaptiveTessellation::set_edge_length_measurement(const EDGE_LEN_TYPE edge_len_type)
{
    switch (edge_len_type) {
    case EDGE_LEN_TYPE::LINEAR2D:
        mesh_parameters.m_get_length = [&](const size_t& vid1, const size_t& vid2) -> double {
            return this->get_length2d(vid1, vid2);
        };
        mesh_parameters.m_accuracy = 0;
        break;
    case EDGE_LEN_TYPE::LINEAR3D:
        mesh_parameters.m_get_length = [&](const size_t& vid1, const size_t& vid2) -> double {
            return this->get_length3d(vid1, vid2);
        };
        mesh_parameters.m_accuracy = 0;
        break;
    case EDGE_LEN_TYPE::N_IMPLICIT_POINTS:
        mesh_parameters.m_get_length = [&](const size_t& vid1, const size_t& vid2) -> double {
            return this->get_length_n_implicit_points(vid1, vid2);
        };
        mesh_parameters.m_accuracy = 0;
        break;
    case EDGE_LEN_TYPE::PT_PER_PIXEL:
        mesh_parameters.m_get_length = [&](const size_t& vid1, const size_t& vid2) -> double {
            return this->get_length_1ptperpixel(vid1, vid2);
        };
        mesh_parameters.m_accuracy = 0;
        break;
    case EDGE_LEN_TYPE::MIPMAP:
        mesh_parameters.m_get_length = [&](const size_t& vid1, const size_t& vid2) -> double {
            return this->get_length_mipmap(vid1, vid2);
        };
        mesh_parameters.m_accuracy = 0;
        break;
    case EDGE_LEN_TYPE::ACCURACY:
        mesh_parameters.m_get_length = [&](const size_t& vid1, const size_t& vid2) -> double {
            return this->get_accuracy_error(vid1, vid2);
        };
        mesh_parameters.m_accuracy = 1;

        break;
    }
}

void AdaptiveTessellation::set_image_function(
    const wmtk::Image& image,
    const WrappingMode wrapping_mode)
{
    mesh_parameters.m_wrapping_mode = wrapping_mode;
    mesh_parameters.m_image = image;
    mesh_parameters.m_get_z = [this](const DScalar& u, const DScalar& v) -> DScalar {
        return this->mesh_parameters.m_image.get(u, v);
    };
    mesh_parameters.m_image_get_coordinate =
        [this](const double& x, const double& y) -> std::pair<int, int> {
        auto [xx, yy] = this->mesh_parameters.m_image.get_pixel_index(x, y);
        return {
            this->mesh_parameters.m_image.get_coordinate(xx, this->mesh_parameters.m_wrapping_mode),
            this->mesh_parameters.m_image.get_coordinate(
                yy,
                this->mesh_parameters.m_wrapping_mode)};
    };
    mesh_parameters.m_mipmap = wmtk::MipMap(image);
    mesh_parameters.m_mipmap.set_wrapping_mode(wrapping_mode);
}

void AdaptiveTessellation::set_displacement()
{
    // needs to be called after m_image is initiated
    // can be also set depending on a user parameter that initialize different Displacement type
    std::shared_ptr<Displacement> displacement_ptr =
        std::make_shared<DisplacementBicubic>(mesh_parameters.m_image);
    mesh_parameters.m_displacement = displacement_ptr;
}

void AdaptiveTessellation::set_projection()
{
    struct Data
    {
        RowMatrix2<Index> E;
        RowMatrix2<Scalar> V;
        lagrange::bvh::EdgeAABBTree<RowMatrix2<Scalar>, RowMatrix2<Index>, 2> aabb;
    };
    auto data = std::make_shared<Data>();
    data->E = get_bnd_edge_matrix();
    RowMatrix2<Scalar> V_aabb = Eigen::MatrixXd::Zero(vert_capacity(), 2);
    for (int i = 0; i < vert_capacity(); ++i) {
        V_aabb.row(i) << vertex_attrs[i].pos[0], vertex_attrs[i].pos[1];
    }
    data->V = V_aabb;
    data->aabb =
        lagrange::bvh::EdgeAABBTree<RowMatrix2<Scalar>, RowMatrix2<Index>, 2>(data->V, data->E);
    std::function<Eigen::RowVector2d(const Eigen::RowVector2d& p)> projection =
        [data = std::move(data)](const Eigen::RowVector2d& p) -> Eigen::RowVector2d {
        uint64_t ind = 0;
        double distance = 0.0;
        Eigen::RowVector2d p_ret;
        assert(data != nullptr);
        (data->aabb).get_closest_point(p, ind, p_ret, distance);
        return p_ret;
    };

    mesh_parameters.m_get_closest_point = std::move(projection);
}

bool AdaptiveTessellation::invariants(const std::vector<Tuple>& new_tris)
{
    if (mesh_parameters.m_has_envelope) {
        for (auto& t : new_tris) {
            std::array<Eigen::Vector3d, 3> tris;
            auto vs = oriented_tri_vertices(t);
            for (auto j = 0; j < 3; j++) {
                tris[j] << vertex_attrs[vs[j].vid(*this)].pos(0),
                    vertex_attrs[vs[j].vid(*this)].pos(1), 0.0;
            }
            if (mesh_parameters.m_envelope.is_outside(tris)) {
                wmtk::logger().debug("envelop invariant fail");
                return false;
            }
        }
    }

    for (auto& t : new_tris) {
        Eigen::Vector2d a, b, c;
        auto verts = oriented_tri_vertices(t);
        assert(verts.size() == 3);
        a = vertex_attrs[verts[0].vid(*this)].pos;
        b = vertex_attrs[verts[1].vid(*this)].pos;
        c = vertex_attrs[verts[2].vid(*this)].pos;

        // check both inverted and exact colinear
        if (wmtk::orient2d_t(a, b, c) != 1) {
            wmtk::logger().debug(
                "----{} false in orientation and collinear {}",
                t.fid(*this),
                wmtk::orient2d_t(a, b, c));
            wmtk::logger().debug("{} {} {}", a, b, c);
            // write_ply("rejected_split_" + std::to_string(t.fid(*this)) + ".ply");
            return false;
        }

        // // add area check (degenerate tirangle)
        // Eigen::Vector3d A, B, C;
        // A.topRows(2) = a;
        // A(2) = 0.;
        // B.topRows(2) = b;
        // B(2) = 0.;
        // C.topRows(2) = c;
        // C(2) = 0.;

        // double area = ((B - A).cross(C - A)).squaredNorm();
        // if (area < 1e-6 * ((B - A).squaredNorm() + (C - A).squaredNorm() + (B -
        // C).squaredNorm())) {
        //     wmtk::logger().info("====false in area ");
        //     wmtk::logger().info("{} {} {}", a, b, c);
        //     return false; // arbitrary chosen previous std::numeric_limits<double>::denorm_min() is
        //                   // too small}
        // }
    }
    return true;
}
std::vector<AdaptiveTessellation::Tuple> AdaptiveTessellation::new_edges_after(
    const std::vector<AdaptiveTessellation::Tuple>& tris) const
{
    std::vector<AdaptiveTessellation::Tuple> new_edges;

    for (auto t : tris) {
        for (auto j = 0; j < 3; j++) {
            new_edges.push_back(tuple_from_edge(t.fid(*this), j));
        }
    }
    wmtk::unique_edge_tuples(*this, new_edges);
    return new_edges;
}

void AdaptiveTessellation::create_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
    std::vector<Eigen::Vector3d> V_env;
    V_env.resize(V.rows());
    std::vector<Eigen::Vector3i> F_env;
    F_env.resize(F.rows());
    // Register attributes
    p_vertex_attrs = &vertex_attrs;
    // Convert from eigen to internal representation (TODO: move to utils and remove it from all
    // app)
    std::vector<std::array<size_t, 3>> tri(F.rows());

    for (int i = 0; i < F.rows(); i++) {
        F_env[i] << (size_t)F(i, 0), (size_t)F(i, 1), (size_t)F(i, 2);
        for (int j = 0; j < 3; j++) {
            tri[i][j] = (size_t)F(i, j);
        }
    }
    // Initialize the trimesh class which handles connectivity
    wmtk::TriMesh::create_mesh(V.rows(), tri);
    // Save the vertex position in the vertex attributes
    for (unsigned i = 0; i < V.rows(); ++i) {
        vertex_attrs[i].pos << V.row(i)[0], V.row(i)[1];
        V_env[i] << V.row(i)[0], V.row(i)[1], 0.0;
    }
    for (auto tri : this->get_faces()) {
        assert(!is_inverted(tri));
    }
    // construct the boundary map for boundary parametrization
    if (mesh_parameters.m_boundary_parameter) mesh_parameters.m_boundary.construct_boudaries(V, F);

    // mark boundary vertices as boundary_vertex
    // but this is not indiscriminatively rejected for all operations
    // other operations are conditioned on whether m_bnd_freeze is turned on
    // also obtain the boudnary parametrizatin too
    for (auto v : this->get_vertices()) {
        if (is_boundary_vertex(v)) {
            vertex_attrs[v.vid(*this)].boundary_vertex = is_boundary_vertex(v);
            set_feature(v);
            vertex_attrs[v.vid(*this)].curve_id = 0; /// only one connected mesh for now
            vertex_attrs[v.vid(*this)].t =
                mesh_parameters.m_boundary.uv_to_t(vertex_attrs[v.vid(*this)].pos);
        }
    }

    for (auto v : get_vertices()) {
        assert(vertex_attrs[v.vid(*this)].t >= 0);
    }

    for (auto v : get_vertices()) {
        if (is_boundary_vertex(v))
            assert(
                (vertex_attrs[v.vid(*this)].pos - mesh_parameters.m_boundary.t_to_uv(
                                                      vertex_attrs[v.vid(*this)].curve_id,
                                                      vertex_attrs[v.vid(*this)].t))
                    .squaredNorm() < 1e-8);
    }
}

Eigen::Matrix<uint64_t, Eigen::Dynamic, 2, Eigen::RowMajor>
AdaptiveTessellation::get_bnd_edge_matrix()
{
    int num_bnd_edge = 0;
    for (auto e : get_edges()) {
        if (is_boundary_edge(e)) num_bnd_edge++;
    }
    Eigen::Matrix<uint64_t, Eigen::Dynamic, 2, Eigen::RowMajor> E(num_bnd_edge, 2);
    int i = 0;
    for (auto e : get_edges()) {
        if (is_boundary_edge(e)) {
            E.row(i) << (uint64_t)e.vid(*this), (uint64_t)e.switch_vertex(*this).vid(*this);
            i++;
        }
    }
    return E;
}

void AdaptiveTessellation::export_mesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    V = Eigen::MatrixXd::Zero(vert_capacity(), 2);
    for (auto& t : get_vertices()) {
        auto i = t.vid(*this);
        V.row(i) = vertex_attrs[i].pos;
    }

    F = Eigen::MatrixXi::Constant(tri_capacity(), 3, -1);
    for (auto& t : get_faces()) {
        auto i = t.fid(*this);
        auto vs = oriented_tri_vertices(t);
        for (int j = 0; j < 3; j++) {
            F(i, j) = vs[j].vid(*this);
        }
    }
}

void AdaptiveTessellation::write_obj(const std::string& path)
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    export_mesh(V, F);

    Eigen::MatrixXd V3 = Eigen::MatrixXd::Zero(V.rows(), 3);
    V3.leftCols(2) = V;

    igl::writeOBJ(path, V3, F);
}

void AdaptiveTessellation::write_ply(const std::string& path)
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    export_mesh(V, F);

    Eigen::MatrixXd V3 = Eigen::MatrixXd::Zero(V.rows(), 3);
    V3.leftCols(2) = V;

    igl::writePLY(path, V3, F);
}

void AdaptiveTessellation::write_vtk(const std::string& path)
{
    std::vector<double> points;
    std::vector<int> elements;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    export_mesh(V, F);
    for (int i = 0; i < V.rows(); i++) {
        auto p = mesh_parameters.m_project_to_3d(V(i, 0), V(i, 1));
        points.emplace_back(p(0));
        points.emplace_back(p(1));
        points.emplace_back(p(2));
    }
    assert(points.size() == 3 * V.rows());

    for (auto e : get_edges()) {
        elements.emplace_back(e.vid(*this));
        elements.emplace_back(e.switch_vertex(*this).vid(*this));
    }
    assert(elements.size() == 2 * get_edges().size());
    // vector<double> scalar_field = {0., 1., 2.};
    // vector<double> vector_field = points;
    const int dim = 3;
    const int cell_size = 2;
    leanvtk::VTUWriter writer;

    std::vector<double> scalar_field;
    for (auto e : get_edges()) {
        if (!e.is_valid(*this)) continue;
        auto cost = mesh_parameters.m_get_length(e.vid(*this), e.switch_vertex(*this).vid(*this));
        // Eigen::Matrix<double, 2, 1> pos1 = vertex_attrs[e.vid(*this)].pos;
        // Eigen::Matrix<double, 2, 1> pos2 = vertex_attrs[e.switch_vertex(*this).vid(*this)].pos;
        // Eigen::Matrix<double, 2, 1> posnew = (pos1 + pos2) * 0.5;
        // cost -=
        //     (mesh_parameters.m_displacement->get_error_per_edge(pos1, posnew) +
        //      mesh_parameters.m_displacement->get_error_per_edge(posnew, pos2));
        scalar_field.emplace_back(cost);
    }
    writer.add_cell_scalar_field("scalar_field", scalar_field);
    // writer.add_vector_field("vector_field", vector_field, dim);

    writer.write_surface_mesh(path, dim, cell_size, points, elements);
}

void AdaptiveTessellation::write_displaced_obj(
    const std::string& path,
    const std::function<double(double, double)>& displacement)
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    export_mesh(V, F);

    Eigen::MatrixXd V3 = Eigen::MatrixXd::Zero(V.rows(), 3);
    for (int i = 0; i < V.rows(); i++) {
        V3.row(i) << V(i, 0), V(i, 1), displacement(V(i, 0), V(i, 1));
    }

    igl::writeOBJ(path, V3, F);
}

void AdaptiveTessellation::write_displaced_obj(
    const std::string& path,
    const std::function<Eigen::Vector3d(double, double)>& displacement)
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    export_mesh(V, F);

    Eigen::MatrixXd V3 = Eigen::MatrixXd::Zero(V.rows(), 3);
    for (int i = 0; i < V.rows(); i++) {
        V3.row(i) = displacement(V(i, 0), V(i, 1));
    }

    igl::writeOBJ(path, V3, F);
}

double AdaptiveTessellation::get_length2d(const size_t& v1, const size_t& v2) const
{
    return (vertex_attrs[v1].pos - vertex_attrs[v2].pos).stableNorm();
}

double AdaptiveTessellation::get_length3d(const size_t& v1, const size_t& v2) const
{
    auto v13d = mesh_parameters.m_project_to_3d(vertex_attrs[v1].pos(0), vertex_attrs[v1].pos(1));
    auto v23d = mesh_parameters.m_project_to_3d(vertex_attrs[v2].pos(0), vertex_attrs[v2].pos(1));
    return (v13d - v23d).stableNorm();
}

double AdaptiveTessellation::get_length_n_implicit_points(const size_t& vid1, const size_t& vid2)
    const
{
    auto v12d = vertex_attrs[vid1].pos;
    auto v22d = vertex_attrs[vid2].pos;

    double length = 0.0;

    // add 3d displacement. add n implicit points to approximate quadrature of the curve
    std::vector<Eigen::Vector3d> quadrature;
    for (int i = 0; i < 6; i++) {
        auto tmp_v2d = v12d * (5 - i) / 5. + v22d * i / 5.;
        quadrature.emplace_back(mesh_parameters.m_project_to_3d(tmp_v2d(0), tmp_v2d(1)));
    }
    for (int i = 0; i < 5; i++) {
        length += (quadrature[i] - quadrature[i + 1]).stableNorm();
    }
    return length;
}

double AdaptiveTessellation::get_length_1ptperpixel(const size_t& vid1, const size_t& vid2) const
{
    auto v12d = vertex_attrs[vid1].pos;
    auto v22d = vertex_attrs[vid2].pos;

    double length = 0.0;
    // get the pixel index of p1 and p2
    auto [xx1, yy1] = mesh_parameters.m_image_get_coordinate(v12d(0), v12d(1));
    auto [xx2, yy2] = mesh_parameters.m_image_get_coordinate(v22d(0), v22d(1));
    // get all the pixels in between p1 and p2
    auto pixel_num = std::max(abs(xx2 - xx1), abs(yy2 - yy1));
    // sum up the length
    // add 3d displacement. add n implicit points to approximate quadrature of the curve
    if (pixel_num <= 0) return -1.;
    assert(pixel_num > 0);
    std ::vector<Eigen::Vector3d> quadrature;
    for (int i = 0; i < (pixel_num + 1); i++) {
        const double u = static_cast<double>(i) / pixel_num;
        auto tmp_v2d = v12d * (1. - u) + v22d * u;
        quadrature.emplace_back(mesh_parameters.m_project_to_3d(tmp_v2d(0), tmp_v2d(1)));
    }
    for (int i = 0; i < pixel_num; i++) {
        length += (quadrature[i] - quadrature[i + 1]).stableNorm();
    }
    return length;
}

double AdaptiveTessellation::get_length_mipmap(const size_t& vid1, const size_t& vid2) const
{
    auto v12d = vertex_attrs[vid1].pos;
    auto v22d = vertex_attrs[vid2].pos;
    int idx = 0;
    int pixel_num = -1;

    std::tie(idx, pixel_num) = mesh_parameters.m_mipmap.get_mipmap_level_pixelnum(v12d, v22d);
    auto image = mesh_parameters.m_mipmap.get_image(idx);
    double length = 0.0;

    if (pixel_num <= 0) return -1.;
    // sum up the length
    // add 3d displacement. add n implicit points to approximate quadrature of the curve
    std::vector<Eigen::Vector3d> quadrature;
    for (int i = 0; i < (pixel_num + 1); i++) {
        auto tmp_v2d = v12d * (pixel_num - i) / pixel_num + v22d * i / pixel_num;
        double z = image.get(tmp_v2d(0), tmp_v2d(1));
        quadrature.emplace_back(tmp_v2d(0), tmp_v2d(1), z);
    }
    for (int i = 0; i < pixel_num; i++) {
        length += (quadrature[i] - quadrature[i + 1]).stableNorm();
    }
    return length;
}

double AdaptiveTessellation::get_quality(const Tuple& loc, int idx) const
{
    // Global ids of the vertices of the triangle
    auto its = oriented_tri_vids(loc);

    // Temporary variable to store the stacked coordinates of the triangle
    std::array<double, 6> T;
    auto energy = -1.;
    for (auto k = 0; k < 3; k++)
        for (auto j = 0; j < 2; j++) T[k * 2 + j] = vertex_attrs[its[k]].pos[j];

    // Energy evaluation
    // energy = wmtk::AMIPS2D_energy(T);
    wmtk::State state = {};
    state.input_triangle = T;
    state.scaling = mesh_parameters.m_target_l;
    state.idx = idx;
    mesh_parameters.m_energy->eval(state);
    energy = state.value;

    // Filter for numerical issues
    if (std::isinf(energy) || std::isnan(energy)) return mesh_parameters.MAX_ENERGY;

    return energy;
}

double AdaptiveTessellation::get_accuracy_error(const size_t& vid1, const size_t& vid2) const
{
    auto v12d = vertex_attrs[vid1].pos;
    auto v22d = vertex_attrs[vid2].pos;

    return mesh_parameters.m_displacement->get_error_per_edge(v12d, v22d);
}

double AdaptiveTessellation::get_accuracy_error(
    const Eigen::Vector2d& p1,
    const Eigen::Vector2d& p2) const
{
    std::function<double(const double&, const double&)> get_z = [&](const double& u,
                                                                    const double& v) -> double {
        return mesh_parameters.m_project_to_3d(u, v)(2);
    };
    auto v1z = get_z(p1(0), p1(1));
    auto v2z = get_z(p2(0), p2(1));
    // get the pixel index of p1 and p2
    auto [xx1, yy1] = mesh_parameters.m_image_get_coordinate(p1(0), p1(1));
    auto [xx2, yy2] = mesh_parameters.m_image_get_coordinate(p2(0), p2(1));
    // get all the pixels in between p1 and p2
    auto pixel_num = std::max(abs(xx2 - xx1), abs(yy2 - yy1));
    if (pixel_num <= 0) return 0.;
    assert(pixel_num > 0);
    double error = 0.0;
    for (int i = 0; i < pixel_num; i++) {
        const double r0 = static_cast<double>(i) / pixel_num;
        auto tmp_p1 = p1 * (1. - r0) + p2 * r0;
        auto tmp_v1z = v1z * (1. - r0) + v2z * r0;
        const double r1 = static_cast<double>(i + 1) / pixel_num;
        auto tmp_p2 = p1 * (1. - r1) + p2 * r1;
        auto tmp_v2z = v1z * (1. - r1) + v2z * r1;
        Eigen::Matrix<double, 2, 3> edge_verts;
        edge_verts.row(0) << tmp_p1(0), tmp_p1(1), tmp_v1z;
        edge_verts.row(1) << tmp_p2(0), tmp_p2(1), tmp_v2z;
        LineQuadrature quad;
        auto displaced_pixel_error =
            quadrature_error_1pixel_eval<double, 5>(edge_verts, get_z, quad);
        error += displaced_pixel_error;
    }
    assert(error >= 0);
    return error;
}

std::pair<double, Eigen::Vector2d> AdaptiveTessellation::get_one_ring_energy(const Tuple& loc) const
{
    auto one_ring = get_one_ring_tris_for_vertex(loc);
    wmtk::DofVector dofx;
    if (is_boundary_vertex(loc) && mesh_parameters.m_boundary_parameter) {
        dofx.resize(1);
        dofx[0] = vertex_attrs[loc.vid(*this)].t; // t
    } else {
        dofx.resize(2); // uv;
        dofx = vertex_attrs[loc.vid(*this)].pos;
    }
    wmtk::NewtonMethodInfo nminfo;
    nminfo.curve_id = vertex_attrs[loc.vid(*this)].curve_id;
    nminfo.target_length = mesh_parameters.m_target_l;
    nminfo.neighbors.resize(one_ring.size(), 4);

    auto is_inverted_coordinates = [this, &loc](auto& A, auto& B) {
        auto res = igl::predicates::orient2d(A, B, this->vertex_attrs[loc.vid(*this)].pos);
        if (res != igl::predicates::Orientation::POSITIVE)
            return true;
        else
            return false;
    };

    for (auto i = 0; i < one_ring.size(); i++) {
        auto tri = one_ring[i];
        if (is_inverted(tri)) {
            return {std::numeric_limits<double>::max(), Eigen::Vector2d::Zero()};
        }
        auto local_tuples = oriented_tri_vertices(tri);
        for (auto j = 0; j < 3; j++) {
            if (local_tuples[j].vid(*this) == loc.vid(*this)) {
                auto v2 = vertex_attrs[local_tuples[(j + 1) % 3].vid(*this)].pos;
                auto v3 = vertex_attrs[local_tuples[(j + 2) % 3].vid(*this)].pos;
                nminfo.neighbors.row(i) << v2(0), v2(1), v3(0), v3(1);
                assert(!is_inverted_coordinates(
                    v2,
                    v3)); // sanity check, no inversion should be heres
            }
        }
    }
    assert(one_ring.size() == nminfo.neighbors.rows());
    auto total_energy = 0.;
    Eigen::Vector2d total_gradient;
    total_gradient.setZero(2);
    for (auto i = 0; i < one_ring.size(); i++) {
        // set State
        // pass the state energy
        State state = {};
        state.two_opposite_vertices = nminfo.neighbors.row(i);
        state.dofx = dofx;
        state.scaling = nminfo.target_length;
        if (mesh_parameters.m_boundary_parameter) {
            assert(mesh_parameters.m_boundary.m_arclengths.size() > 0);
            assert(mesh_parameters.m_boundary.m_boundaries.size() > 0);
        }
        DofsToPositions dofs_to_pos(mesh_parameters.m_boundary, nminfo.curve_id);
        mesh_parameters.m_energy->eval(state, dofs_to_pos);
        total_energy += state.value;
        total_gradient += state.gradient;
    }
    return {total_energy, total_gradient};
}

Eigen::VectorXd AdaptiveTessellation::get_quality_all_triangles()
{
    // Use a concurrent vector as for_each_face is parallel
    tbb::concurrent_vector<double> quality;
    quality.reserve(vertex_attrs.size());

    // Evaluate quality in parallel
    for_each_face([&](auto& f) { quality.push_back(get_quality(f)); });

    // Copy back in a VectorXd
    Eigen::VectorXd ret(quality.size());
    for (unsigned i = 0; i < quality.size(); ++i) ret[i] = quality[i];
    return ret;
}

bool AdaptiveTessellation::is_inverted(const Tuple& loc) const
{
    // Get the vertices ids
    auto vs = oriented_tri_vertices(loc);

    igl::predicates::exactinit();

    // Use igl for checking orientation
    auto res = igl::predicates::orient2d(
        vertex_attrs[vs[0].vid(*this)].pos,
        vertex_attrs[vs[1].vid(*this)].pos,
        vertex_attrs[vs[2].vid(*this)].pos);
    // The element is inverted if it not positive (i.e. it is negative or it is degenerate)
    return (res != igl::predicates::Orientation::POSITIVE);
}

void AdaptiveTessellation::mesh_improvement(int max_its)
{
    auto start_time = lagrange::get_timestamp();

    [[maybe_unused]] igl::Timer timer;
    [[maybe_unused]] double avg_len = 0.0;
    [[maybe_unused]] double pre_avg_len = 0.0;
    [[maybe_unused]] double pre_max_energy = -1.0;
    [[maybe_unused]] double old_average = 1e-4;
    wmtk::logger().info("target len {}", mesh_parameters.m_target_l);
    wmtk::logger().info("current length {}", avg_edge_len(*this));
    // mesh_parameters.js_log["edge_length_avg_start"] = avg_edge_len(*this);
    for (int it = 0; it < max_its; it++) {
        wmtk::logger().info("\n========it {}========", it);

        ///energy check
        wmtk::logger().info(
            "current max energy {} stop energy {}",
            mesh_parameters.m_max_energy,
            mesh_parameters.m_stop_energy);
        wmtk::logger().info("current length {}", avg_edge_len(*this));

        split_all_edges();
        assert(invariants(get_faces()));
        auto split_finish_time = lagrange::get_timestamp();
        mesh_parameters.js_log["iteration_" + std::to_string(it)]["split time"] =
            lagrange::timestamp_diff_in_seconds(start_time, split_finish_time);
        consolidate_mesh();
        write_displaced_obj(
            mesh_parameters.m_output_folder + "/after_split_" + std::to_string(it) + ".obj",
            mesh_parameters.m_project_to_3d);

        // collapse_all_edges();
        // assert(invariants(get_faces()));
        // consolidate_mesh();
        // write_displaced_obj(
        //     mesh_parameters.m_output_folder + "/after_collapse_" + std::to_string(it) + ".obj",
        //     mesh_parameters.m_project_to_3d);
        split_finish_time = lagrange::get_timestamp();
        swap_all_edges();
        assert(invariants(get_faces()));
        auto swap_finish_time = lagrange::get_timestamp();
        mesh_parameters.js_log["iteration_" + std::to_string(it)]["swap time"] =
            lagrange::timestamp_diff_in_seconds(split_finish_time, swap_finish_time);
        consolidate_mesh();
        write_displaced_obj(
            mesh_parameters.m_output_folder + "/after_swap_" + std::to_string(it) + ".obj",
            mesh_parameters.m_project_to_3d);

        swap_finish_time = lagrange::get_timestamp();
        smooth_all_vertices();
        assert(invariants(get_faces()));
        auto smooth_finish_time = lagrange::get_timestamp();
        mesh_parameters.js_log["iteration_" + std::to_string(it)]["smooth time"] =
            lagrange::timestamp_diff_in_seconds(swap_finish_time, smooth_finish_time);
        consolidate_mesh();
        write_displaced_obj(
            mesh_parameters.m_output_folder + "/after_smooth_" + std::to_string(it) + ".obj",
            mesh_parameters.m_project_to_3d);

        auto avg_grad = (mesh_parameters.m_gradient / vert_capacity()).stableNorm();

        wmtk::logger().info(
            "++++++++v {} t {} avg gradient {}++++++++",
            vert_capacity(),
            tri_capacity(),
            avg_grad);

        mesh_parameters.js_log["iteration_" + std::to_string(it)]["avg_grad"] = avg_grad;
        mesh_parameters.js_log["iteration_" + std::to_string(it)]["num_v"] = vert_capacity();
        mesh_parameters.js_log["iteration_" + std::to_string(it)]["num_f"] = tri_capacity();
        mesh_parameters.js_log["iteration_" + std::to_string(it)]["energy_max"] =
            mesh_parameters.m_max_energy;
        mesh_parameters.js_log["iteration_" + std::to_string(it)]["edge_len_avg"] =
            avg_edge_len(*this);
        mesh_parameters.js_log["iteration_" + std::to_string(it)]["edge_len_target"] =
            mesh_parameters.m_target_l;
        if (avg_grad < 1e-4) {
            wmtk::logger().info(
                "!!!avg grad is less than 1e-4 !!! energy doesn't improve anymore. early stop"
                "itr {}, avg length {} ",
                it,
                avg_len);
            break;
        }
        mesh_parameters.m_gradient = Eigen::Vector2d(0., 0.);
        avg_len = avg_edge_len(*this);
        mesh_parameters.js_log["iteration_" + std::to_string(it)]["edge_len_avg_final"] = avg_len;
        if (abs(avg_grad - old_average) < 1e-4) break;
        old_average = avg_grad;
        if (mesh_parameters.m_target_l <= 0 &&
            mesh_parameters.m_max_energy < mesh_parameters.m_stop_energy) {
            break;
        }
        pre_avg_len = avg_len;
        pre_max_energy = mesh_parameters.m_max_energy;
        consolidate_mesh();
    }

    wmtk::logger().info(
        "/////final: max energy {} , avg len {} ",
        mesh_parameters.m_max_energy,
        avg_len);
    consolidate_mesh();
}
void AdaptiveTessellation::flatten_dofs(Eigen::VectorXd& v_flat)
{
    auto verts = get_vertices();
    v_flat.resize(verts.size() * 2);
    for (auto v : verts) {
        if (is_boundary_vertex(v) && mesh_parameters.m_boundary_parameter) {
            v_flat(v.vid(*this) * 2) = vertex_attrs[v.vid(*this)].t;
            v_flat(v.vid(*this) * 2 + 1) = std::numeric_limits<double>::infinity();
        } else {
            v_flat(v.vid(*this) * 2) = vertex_attrs[v.vid(*this)].pos(0);
            v_flat(v.vid(*this) * 2 + 1) = vertex_attrs[v.vid(*this)].pos(1);
        }
    }
}

// get the energy defined by edge_length_energy over each face of the mesh
// assuming the vert_capacity() == get_vertices.size()
double AdaptiveTessellation::get_mesh_energy(const Eigen::VectorXd& v_flat)
{
    double total_energy = 0;
    Eigen::MatrixXd energy_matrix;
    energy_matrix.resize(get_faces().size(), 2);
    for (auto& face : get_faces()) {
        // wmtk::logger().info("getting energy on {} ", f_cnt++);
        auto verts = oriented_tri_vertices(face);
        Eigen::Matrix3d v_matrix;
        v_matrix.setZero(3, 3);
        for (int i = 0; i < 3; i++) {
            auto vert = verts[i];
            if (is_boundary_vertex(vert) && mesh_parameters.m_boundary_parameter) {
                auto uv = mesh_parameters.m_boundary.t_to_uv(
                    vertex_attrs[vert.vid(*this)].curve_id,
                    v_flat[vert.vid(*this) * 2]);
                v_matrix.row(i) = mesh_parameters.m_project_to_3d(uv(0), uv(1));
            } else {
                auto u = v_flat[vert.vid(*this) * 2];
                auto v = v_flat[vert.vid(*this) * 2 + 1];
                v_matrix.row(i) = mesh_parameters.m_project_to_3d(u, v);
            }
        }

        assert(mesh_parameters.m_target_l != 0);
        double tri_energy = 0.;
        auto BA = v_matrix.row(1) - v_matrix.row(0);
        auto CA = v_matrix.row(2) - v_matrix.row(0);
        auto BC = v_matrix.row(1) - v_matrix.row(2);
        tri_energy += pow(BA.squaredNorm() - pow(mesh_parameters.m_target_l, 2), 2);
        tri_energy += pow(BC.squaredNorm() - pow(mesh_parameters.m_target_l, 2), 2);
        tri_energy += pow(CA.squaredNorm() - pow(mesh_parameters.m_target_l, 2), 2);

        energy_matrix(face.fid(*this), 0) = tri_energy;
        double area = (BA.cross(CA)).squaredNorm();
        double A_hat = 0.5 * (std::sqrt(3) / 2) * 0.5 *
                       pow(mesh_parameters.m_target_l, 2); // this is arbitrary now
        assert(A_hat > 0);
        if (area <= 0) {
            tri_energy += std::numeric_limits<double>::infinity();
        }
        if (area < A_hat) {
            assert((area / A_hat) < 1.0);
            tri_energy += -(area - A_hat) * (area - A_hat) * log(area / A_hat);
        }
        total_energy += tri_energy;

        energy_matrix(face.fid(*this), 1) = tri_energy;
    }
    igl::writeDMAT("mesh_energy.dmat", energy_matrix);

    return total_energy;
}
/// debugging
void AdaptiveTessellation::gradient_debug(int max_its)
{
    split_all_edges();
    assert(invariants(get_faces()));
    consolidate_mesh();
    write_obj("before_smooth.obj");
    double old_average = 1e-4;
    for (int it = 0; it < max_its; it++) {
        wmtk::logger().info("\n========it {}========", it);
        wmtk::logger().info("current length {}", avg_edge_len(*this));

        smooth_all_vertices();
        assert(invariants(get_faces()));
        consolidate_mesh();
        write_displaced_obj(
            "smooth_" + std::to_string(it) + ".obj",
            mesh_parameters.m_project_to_3d);

        wmtk::logger().info(
            "++++++++v {} t {} avg gradient {}++++++++",
            vert_capacity(),
            tri_capacity(),
            mesh_parameters.m_gradient / vert_capacity());
        auto avg_grad = (mesh_parameters.m_gradient / vert_capacity()).stableNorm();

        mesh_parameters.js_log["iteration_" + std::to_string(it)]["avg_grad"] = avg_grad;
        Eigen::VectorXd v_flat;
        flatten_dofs(v_flat);
        mesh_parameters.js_log["iteration_" + std::to_string(it)]["energy"] =
            get_mesh_energy(v_flat);
        if (abs(avg_grad - old_average) < 1e-5) break;
        old_average = avg_grad;
        mesh_parameters.m_gradient = Eigen::Vector2d(0., 0.);
    }
}
} // namespace adaptive_tessellation