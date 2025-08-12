#include "Operation.hpp"
#include <map>
#include <unordered_map>
#include <unordered_set>
#ifdef WMTK_RECORD_OPERATIONS
#include <wmtk/Record_Operations.hpp>
#endif

#include <nlohmann/json.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include <wmtk/simplex/IdSimplexCollection.hpp>
#include <wmtk/simplex/closed_star_iterable.hpp>
#include <wmtk/utils/mesh_intersection_utils.hpp>

#include <wmtk/operations/utils/local_joint_flatten.hpp>
#include <wmtk/simplex/closed_star.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/TupleInspector.hpp>
#include <wmtk/utils/orient.hpp>

// it's ugly but for teh visitor we need these included
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/utils/TetMesh_Embedding.hpp>
using json = nlohmann::json;

// for Debugging output
#include <igl/Timer.h>
#include <igl/boundary_facets.h>
#include <igl/boundary_loop.h>
#include <igl/doublearea.h>
#include <igl/harmonic.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/readOBJ.h>
#include <igl/remove_unreferenced.h>
#include <igl/slice.h>
#include <igl/slice_into.h>
#include <igl/triangle/scaf.h>
#include <igl/writeOBJ.h>
// #define WMTK_DEBUG_VISUALIZE
namespace wmtk::operations {


Operation::Operation(Mesh& mesh)
    : m_mesh(mesh)
    , m_invariants(mesh)
{}

Operation::~Operation() = default;


std::shared_ptr<const operations::AttributeTransferStrategyBase> Operation::get_transfer_strategy(
    const attribute::MeshAttributeHandle& attribute)
{
    assert(attribute.is_same_mesh(mesh()));

    for (auto& s : m_attr_transfer_strategies) {
        if (s->matches_attribute(attribute)) return s;
    }

    throw std::runtime_error("unable to find attribute");
}

void Operation::clear_attribute_transfer_strategies()
{
    m_attr_transfer_strategies.clear();
}

void Operation::set_transfer_strategy(
    const attribute::MeshAttributeHandle& attribute,
    const std::shared_ptr<const operations::AttributeTransferStrategyBase>& other)
{
    assert(attribute.is_same_mesh(mesh()));

    for (auto& s : m_attr_transfer_strategies) {
        if (s->matches_attribute(attribute)) {
            s = other;
            return;
        }
    }

    throw std::runtime_error("unable to find attribute");
}

void Operation::add_transfer_strategy(
    const std::shared_ptr<const operations::AttributeTransferStrategyBase>& other)
{
    spdlog::debug("Adding a transfer");
    m_attr_transfer_strategies.emplace_back(other);
}


std::vector<simplex::Simplex> Operation::operator()(const simplex::Simplex& simplex)
{
    // for debugging
    static int succ_embed_count = 0;


    // Make it special for MeshConsolidate
    if (operation_name == "MeshConsolidate") {
        return execute(simplex);
    }
    if (!mesh().is_valid(simplex)) {
        return {};
    }
    if (!before(simplex)) {
        return {};
    }

    const auto simplex_resurrect = simplex;

    auto scope = mesh().create_scope();
    assert(simplex.primitive_type() == primitive_type());


    auto unmods = unmodified_primitives(simplex_resurrect);


    auto mods = execute(simplex_resurrect);


    if (!mods.empty()) { // success should be marked here

        if (operation_name != "MeshConsolidate") apply_attribute_transfer(mods);

        if (after(unmods, mods)) {
            // store local atlas for retrieval
#ifdef WMTK_RECORD_OPERATIONS

            if (m_record && operation_name != "MeshConsolidate") {
                if (succ_operations_count % 100 == 0) {
                    std::cout << "operation id: " << succ_operations_count << "\n";
                }

                // create a local atlas file
                // std::cout << "operation " << operation_name << " is successful\n";
                std::string filename = OperationLogPath + OperationLogPrefix +
                                       std::to_string(succ_operations_count) + ".json";
                std::ofstream operation_log_file(filename);
                json operation_log;

                if (operation_log_file.is_open()) {
                    // Visualize V_before, F_before and V_after, F_after using
                    // libigl, switch with keys 0 and 1
                    auto visualize_meshes = [](const Eigen::MatrixXd& V_before,
                                               const Eigen::MatrixXi& F_before,
                                               const Eigen::MatrixXd& V_after,
                                               const Eigen::MatrixXi& F_after) {
                        igl::opengl::glfw::Viewer viewer;
                        viewer.data().set_mesh(V_before, F_before);

                        viewer.callback_key_down = [&V_before, &F_before, &V_after, &F_after](
                                                       igl::opengl::glfw::Viewer& v,
                                                       unsigned int key,
                                                       int mod) {
                            if (key == '0') {
                                v.data().clear();
                                v.data().set_mesh(V_before, F_before);
                                return true;
                            }
                            if (key == '1') {
                                v.data().clear();
                                v.data().set_mesh(V_after, F_after);
                                return true;
                            }
                            return false;
                        };

                        viewer.launch();
                    };
                    igl::Timer timer;
                    // save operation_name
                    operation_log["operation_name"] = operation_name;

                    // helper function to convert matrix to json
                    auto matrix_to_json = [](const auto& matrix) {
                        json result = json::array();
                        for (int i = 0; i < matrix.rows(); ++i) {
                            json row = json::array();
                            for (int j = 0; j < matrix.cols(); ++j) {
                                row.push_back(matrix(i, j));
                            }
                            result.push_back(row);
                        }
                        return result;
                    };

                    ////////////////////////////
                    // handle triangle mesh
                    ////////////////////////////
                    if (mesh().top_simplex_type() == PrimitiveType::Triangle) {
                        // get local mesh before and after the operation
                        auto [F_after, V_after, id_map_after, v_id_map_after] =
                            utils::get_local_trimesh(static_cast<const TriMesh&>(mesh()), mods[0]);

                        auto [F_before, V_before, id_map_before, v_id_map_before] =
                            mesh().parent_scope(
                                [&](const simplex::Simplex& s) {
                                    if (operation_name == "EdgeCollapse")
                                        return utils::get_local_trimesh_before_collapse(
                                            static_cast<const TriMesh&>(mesh()),
                                            s);
                                    return utils::get_local_trimesh(
                                        static_cast<const TriMesh&>(mesh()),
                                        s);
                                },
                                simplex);


                        auto to_three_cols = [](const Eigen::MatrixXd& V) {
                            if (V.cols() == 2) {
                                Eigen::MatrixXd V_temp(V.rows(), 3);
                                V_temp << V, Eigen::VectorXd::Zero(V.rows());
                                return V_temp;
                            } else {
                                return V;
                            }
                        };

                        if (operation_name == "EdgeCollapse") {
                            // add different cases for different boundary conditions
                            auto [is_bd_v0, is_bd_v1] = mesh().parent_scope(
                                [&](const simplex::Simplex& s) {
                                    return std::make_tuple(
                                        mesh().is_boundary(
                                            simplex::Simplex::vertex(mesh(), s.tuple())),
                                        mesh().is_boundary(simplex::Simplex::vertex(
                                            mesh(),
                                            mesh().switch_tuple(
                                                s.tuple(),
                                                PrimitiveType::Vertex))));
                                },
                                simplex);

                            Eigen::MatrixXd UV_joint;
                            std::vector<int64_t> v_id_map_joint;

                            // TODO: for debugging
                            if (false) {
                                igl::writeOBJ(
                                    OperationLogPath + "/VF_before_" +
                                        std::to_string(succ_operations_count) + ".obj",
                                    to_three_cols(V_before),
                                    F_before);
                                igl::writeOBJ(
                                    OperationLogPath + "/VF_after_" +
                                        std::to_string(succ_operations_count) + ".obj",
                                    to_three_cols(V_after),
                                    F_after);
                                // write v_id_map_before and v_id_map_after to a text file
                                std::ofstream v_id_map_file(
                                    OperationLogPath + "/v_id_map_" +
                                    std::to_string(succ_operations_count) + ".txt");
                                if (v_id_map_file.is_open()) {
                                    v_id_map_file << "boundary case:\n"
                                                  << is_bd_v0 << " " << is_bd_v1 << "\n";
                                    v_id_map_file << "v_id_map_before: \n";
                                    for (auto id : v_id_map_before) {
                                        v_id_map_file << id << " ";
                                    }
                                    v_id_map_file << "\n";
                                    v_id_map_file << "v_id_map_after: \n";
                                    for (auto id : v_id_map_after) {
                                        v_id_map_file << id << " ";
                                    }
                                    v_id_map_file.close();
                                } else {
                                    std::cerr << "unable to open file for writing\n";
                                }
                            }

                            timer.start();
                            utils::local_joint_flatten(
                                F_before,
                                to_three_cols(V_before),
                                v_id_map_before,
                                F_after,
                                to_three_cols(V_after),
                                v_id_map_after,
                                UV_joint,
                                v_id_map_joint,
                                is_bd_v0,
                                is_bd_v1);
                            std::cout << "time for local_joint_flatten: " << timer.getElapsedTime()
                                      << " seconds\n";

                            if (false) {
                                igl::writeOBJ(
                                    OperationLogPath + "/UV_after_" +
                                        std::to_string(succ_operations_count) + ".obj",
                                    to_three_cols(UV_joint),
                                    F_after);
                                igl::writeOBJ(
                                    OperationLogPath + "/UV_before_" +
                                        std::to_string(succ_operations_count) + ".obj",
                                    to_three_cols(UV_joint),
                                    F_before);
                            }
                            operation_log["UV_joint"]["rows"] = UV_joint.rows();
                            operation_log["UV_joint"]["values"] = matrix_to_json(UV_joint);
                            operation_log["v_id_map_joint"] = v_id_map_joint;
                            operation_log["F_after"]["rows"] = F_after.rows();
                            operation_log["F_after"]["values"] = matrix_to_json(F_after);
                            operation_log["F_before"]["rows"] = F_before.rows();
                            operation_log["F_before"]["values"] = matrix_to_json(F_before);
                            operation_log["F_id_map_after"] = id_map_after;
                            operation_log["F_id_map_before"] = id_map_before;

                            // TODO: add an after check here just to make sure this operation
                            // can be localy parametrized without problem
                            Eigen::VectorXd dbarea_before, dbarea_after;
                            igl::doublearea(UV_joint, F_before, dbarea_before);
                            igl::doublearea(UV_joint, F_after, dbarea_after);

                            if (dbarea_before.minCoeff() < 0) {
                                // std::cerr << "negative area in F_before detected\n";
                                throw std::runtime_error(
                                    "collapse negative area in F_before detected");
                                scope.mark_failed();
                                return {};
                            }

                            if (dbarea_after.minCoeff() < 0) {
                                throw std::runtime_error(
                                    "collapse negative area in F_after detected");
                                scope.mark_failed();
                                return {};
                            }

                            if (wmtk::utils::hasSelfIntersection2D(UV_joint, F_before)) {
                                // std::cout << "operation number: " << succ_operations_count <<
                                // "\n"; std::cerr << "self intersection in collapse F_before
                                // detected\n";
                                throw std::runtime_error(
                                    "collapse self intersection in F_before detected");
                                scope.mark_failed();
                                return {};
                            } else {
                                // std::cout << "no self intersection in collapse F_before
                                // detected\n";
                            }


                        } else { // for other operations

                            // add skip option
                            bool skip = false;
                            if (operation_name == "AttributesUpdate") {
                                bool is_bd = mesh().is_boundary(simplex);

                                if (!is_bd) {
                                    Eigen::MatrixXd UV_joint;
                                    timer.start();
                                    utils::local_joint_flatten_smoothing(
                                        F_before,
                                        to_three_cols(V_before),
                                        to_three_cols(V_after),
                                        F_after,
                                        UV_joint);
                                    std::cout << "time for local_joint_flatten_smoothing: "
                                              << timer.getElapsedTime() << " seconds\n";
                                    V_before = UV_joint;
                                    V_after = UV_joint;

                                    v_id_map_before.push_back(v_id_map_before[0]);
                                    v_id_map_after.push_back(v_id_map_after[0]);
                                } else {
                                    skip = true;
                                }
                            }

                            if (operation_name == "TriEdgeSwap") {
                                Eigen::MatrixXd UV_joint;
                                Eigen::VectorXi local_map;
                                utils::local_joint_flatten_swap(
                                    to_three_cols(V_before),
                                    to_three_cols(V_after),
                                    F_before,
                                    F_after,
                                    UV_joint,
                                    local_map);

                                V_before = UV_joint;
                                V_after.resize(V_before.rows(), V_before.cols());
                                for (int i = 0; i < 4; ++i) {
                                    V_after.row(i) = V_before.row(local_map(i));
                                }
                            }

                            if (operation_name == "EdgeSplit") {
                                // Check if mesh coordinates are 2D
                                if (V_before.cols() == 2) {
                                    // For 2D coordinates, do nothing - already in 2D
                                } else if (V_before.cols() == 3) {
                                    // For 3D coordinates, check if it's boundary or interior edge
                                    bool is_boundary_edge = mesh().parent_scope(
                                        [&](const simplex::Simplex& s) {
                                            return mesh().is_boundary(s);
                                        },
                                        simplex);


                                    // visualize_meshes(V_before, F_before, V_after, F_after);
                                    if (is_boundary_edge) {
                                        // Boundary edge split: simple projection to best-fit plane
                                        // Compute centroid for translation
                                        Eigen::Vector3d centroid = V_before.colwise().mean();

                                        // Compute covariance matrix for PCA
                                        Eigen::MatrixXd V_centered =
                                            V_before.rowwise() - centroid.transpose();
                                        Eigen::Matrix3d cov = V_centered.transpose() * V_centered;

                                        // Find the two principal components (largest eigenvalues)
                                        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
                                        Eigen::Matrix3d eigenvectors = solver.eigenvectors();

                                        // Use the two largest eigenvectors as the 2D basis
                                        Eigen::Vector3d u =
                                            eigenvectors.col(2).normalized(); // largest
                                        Eigen::Vector3d v =
                                            eigenvectors.col(1).normalized(); // second largest

                                        // Project to 2D
                                        Eigen::MatrixXd V_before_2d(V_before.rows(), 2);
                                        Eigen::MatrixXd V_after_2d(V_after.rows(), 2);

                                        for (int i = 0; i < V_before.rows(); ++i) {
                                            Eigen::Vector3d p =
                                                V_before.row(i) - centroid.transpose();
                                            V_before_2d.row(i) =
                                                Eigen::Vector2d(p.dot(u), p.dot(v));
                                        }

                                        // Map vertices from after to before using v_id_map
                                        for (int i = 0; i < V_after.rows(); ++i) {
                                            int64_t global_vid_after = v_id_map_after[i];

                                            // Find if this vertex exists in before mesh
                                            auto it = std::find(
                                                v_id_map_before.begin(),
                                                v_id_map_before.end(),
                                                global_vid_after);

                                            if (it != v_id_map_before.end()) {
                                                // Existing vertex: copy 2D position from before
                                                int before_index =
                                                    std::distance(v_id_map_before.begin(), it);
                                                V_after_2d.row(i) = V_before_2d.row(before_index);
                                            } else {
                                                // New vertex: project using same basis vectors
                                                Eigen::Vector3d p =
                                                    V_after.row(i) - centroid.transpose();
                                                V_after_2d.row(i) =
                                                    Eigen::Vector2d(p.dot(u), p.dot(v));
                                            }
                                        }

                                        V_before = V_before_2d;
                                        V_after = V_after_2d;
                                        // visualize_meshes(V_before, F_before, V_after, F_after);
                                    } else {
                                        // Interior edge split: flatten the "book pages" around the shared edge

                                        if (F_before.rows() == 2) {
                                            // Find the shared edge vertices (the "spine" of the book)
                                            std::vector<int> shared_vertices;
                                            for (int i = 0; i < 3; ++i) {
                                                for (int j = 0; j < 3; ++j) {
                                                    if (F_before(0, i) == F_before(1, j)) {
                                                        shared_vertices.push_back(F_before(0, i));
                                                        break;
                                                    }
                                                }
                                            }
                                            
                                            if (shared_vertices.size() != 2) {
                                                throw std::runtime_error("Expected exactly 2 shared vertices for interior edge split");
                                            }
                                            
                                            int spine_v1 = shared_vertices[0];
                                            int spine_v2 = shared_vertices[1];
                                            
                                            // Find the "page" vertices (one from each triangle, not on the spine)
                                            int page_v1 = -1, page_v2 = -1;
                                            for (int i = 0; i < 3; ++i) {
                                                if (F_before(0, i) != spine_v1 && F_before(0, i) != spine_v2) {
                                                    page_v1 = F_before(0, i);
                                                    break;
                                                }
                                            }
                                            for (int i = 0; i < 3; ++i) {
                                                if (F_before(1, i) != spine_v1 && F_before(1, i) != spine_v2) {
                                                    page_v2 = F_before(1, i);
                                                    break;
                                                }
                                            }
                                            
                                            if (page_v1 == -1 || page_v2 == -1) {
                                                throw std::runtime_error("Could not find page vertices for book flattening");
                                            }
                                            
                                            // Create the flattened 2D layout
                                            Eigen::MatrixXd V_before_2d(V_before.rows(), 2);
                                            Eigen::MatrixXd V_after_2d(V_after.rows(), 2);
                                            
                                            // Place spine edge vertically: spine_v1 at (0,0), spine_v2 at (0, edge_length)
                                            double spine_length = (V_before.row(spine_v2) - V_before.row(spine_v1)).norm();
                                            V_before_2d.row(spine_v1) = Eigen::Vector2d(0, 0);
                                            V_before_2d.row(spine_v2) = Eigen::Vector2d(0, spine_length);
                                            
                                            // Helper lambda to place a vertex and ensure positive triangle orientation
                                            auto place_vertex_2d_with_orientation = [&](int vertex_idx, int ref_v1, int ref_v2, 
                                                                                          double ref_x1, double ref_y1, double ref_x2, double ref_y2,
                                                                                          bool ensure_positive_area, const Eigen::MatrixXi& face_containing_vertex, int face_idx) -> Eigen::Vector2d {
                                                double d1 = (V_before.row(vertex_idx) - V_before.row(ref_v1)).norm();
                                                double d2 = (V_before.row(vertex_idx) - V_before.row(ref_v2)).norm();
                                                double baseline_len = std::sqrt((ref_x2 - ref_x1) * (ref_x2 - ref_x1) + (ref_y2 - ref_y1) * (ref_y2 - ref_y1));
                                                
                                                // Use law of cosines to find angle at ref_v1
                                                double cos_angle = (d1 * d1 + baseline_len * baseline_len - d2 * d2) / (2 * d1 * baseline_len);
                                                cos_angle = std::clamp(cos_angle, -1.0, 1.0);
                                                double sin_angle = std::sqrt(1 - cos_angle * cos_angle);
                                                
                                                // Direction vector along the baseline (from ref_v1 to ref_v2)
                                                Eigen::Vector2d baseline_dir = Eigen::Vector2d(ref_x2 - ref_x1, ref_y2 - ref_y1).normalized();
                                                Eigen::Vector2d perp_dir = Eigen::Vector2d(-baseline_dir.y(), baseline_dir.x()); // perpendicular direction
                                                
                                                // Position along baseline
                                                Eigen::Vector2d pos_along = Eigen::Vector2d(ref_x1, ref_y1) + d1 * cos_angle * baseline_dir;
                                                
                                                // Try both sides of the baseline
                                                Eigen::Vector2d pos_positive = pos_along + d1 * sin_angle * perp_dir;
                                                Eigen::Vector2d pos_negative = pos_along - d1 * sin_angle * perp_dir;
                                                
                                                if (!ensure_positive_area) {
                                                    return pos_positive; // Default to positive side
                                                }
                                                
                                                // Check which position gives positive signed area for the triangle
                                                auto compute_signed_area = [](const Eigen::Vector2d& p0, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) -> double {
                                                    return (p1.x() - p0.x()) * (p2.y() - p0.y()) - (p2.x() - p0.x()) * (p1.y() - p0.y());
                                                };
                                                
                                                // Get the triangle vertices in 2D space
                                                Eigen::Vector2d v0_2d, v1_2d, v2_2d;
                                                int v0 = face_containing_vertex(face_idx, 0);
                                                int v1 = face_containing_vertex(face_idx, 1);
                                                int v2 = face_containing_vertex(face_idx, 2);
                                                
                                                // Assign 2D coordinates based on which vertex we're placing
                                                if (v0 == vertex_idx) {
                                                    v1_2d = (v1 == ref_v1) ? Eigen::Vector2d(ref_x1, ref_y1) : Eigen::Vector2d(ref_x2, ref_y2);
                                                    v2_2d = (v2 == ref_v1) ? Eigen::Vector2d(ref_x1, ref_y1) : Eigen::Vector2d(ref_x2, ref_y2);
                                                    
                                                    double area_pos = compute_signed_area(pos_positive, v1_2d, v2_2d);
                                                    double area_neg = compute_signed_area(pos_negative, v1_2d, v2_2d);
                                                    return (area_pos > area_neg) ? pos_positive : pos_negative;
                                                } else if (v1 == vertex_idx) {
                                                    v0_2d = (v0 == ref_v1) ? Eigen::Vector2d(ref_x1, ref_y1) : Eigen::Vector2d(ref_x2, ref_y2);
                                                    v2_2d = (v2 == ref_v1) ? Eigen::Vector2d(ref_x1, ref_y1) : Eigen::Vector2d(ref_x2, ref_y2);
                                                    
                                                    double area_pos = compute_signed_area(v0_2d, pos_positive, v2_2d);
                                                    double area_neg = compute_signed_area(v0_2d, pos_negative, v2_2d);
                                                    return (area_pos > area_neg) ? pos_positive : pos_negative;
                                                } else { // v2 == vertex_idx
                                                    v0_2d = (v0 == ref_v1) ? Eigen::Vector2d(ref_x1, ref_y1) : Eigen::Vector2d(ref_x2, ref_y2);
                                                    v1_2d = (v1 == ref_v1) ? Eigen::Vector2d(ref_x1, ref_y1) : Eigen::Vector2d(ref_x2, ref_y2);
                                                    
                                                    double area_pos = compute_signed_area(v0_2d, v1_2d, pos_positive);
                                                    double area_neg = compute_signed_area(v0_2d, v1_2d, pos_negative);
                                                    return (area_pos > area_neg) ? pos_positive : pos_negative;
                                                }
                                            };
                                            
                                            // Place the first page vertex ensuring positive area for its triangle
                                            V_before_2d.row(page_v1) = place_vertex_2d_with_orientation(page_v1, spine_v1, spine_v2, 0, 0, 0, spine_length, true, F_before, 0);
                                            
                                            // Place the second page vertex ensuring positive area for its triangle
                                            V_before_2d.row(page_v2) = place_vertex_2d_with_orientation(page_v2, spine_v1, spine_v2, 0, 0, 0, spine_length, true, F_before, 1);
                                            
                                            // Map the after mesh vertices to 2D
                                            for (int i = 0; i < V_after.rows(); ++i) {
                                                int64_t global_vid_after = v_id_map_after[i];
                                                auto it = std::find(v_id_map_before.begin(), v_id_map_before.end(), global_vid_after);
                                                
                                                if (it != v_id_map_before.end()) {
                                                    // Existing vertex: copy its 2D position
                                                    int before_index = std::distance(v_id_map_before.begin(), it);
                                                    V_after_2d.row(i) = V_before_2d.row(before_index);
                                                } else {
                                                    // New vertex (split point): place it using the same geometric relationships
                                                    // Since it's a split on the spine edge, it should lie on the spine
                                                    double d1 = (V_after.row(i) - V_before.row(spine_v1)).norm();
                                                    double d2 = (V_after.row(i) - V_before.row(spine_v2)).norm();
                                                    // The new vertex should be positioned proportionally along the spine
                                                    double total_spine = d1 + d2;
                                                    double ratio = d1 / total_spine;
                                                    V_after_2d.row(i) = Eigen::Vector2d(0, ratio * spine_length);
                                                }
                                            }
                                            
                                            V_before = V_before_2d;
                                            V_after = V_after_2d;
                                        }
                                    }
                                }
                            }
                            if (!skip) {
                                // log the mesh before and after the operation
                                operation_log["is_skipped"] = false;

                                operation_log["F_after"]["rows"] = F_after.rows();
                                operation_log["F_after"]["values"] = matrix_to_json(F_after);
                                operation_log["V_after"]["rows"] = V_after.rows();
                                operation_log["V_after"]["values"] = matrix_to_json(V_after);
                                operation_log["F_id_map_after"] = id_map_after;
                                operation_log["V_id_map_after"] = v_id_map_after;

                                operation_log["F_before"]["rows"] = F_before.rows();
                                operation_log["F_before"]["values"] = matrix_to_json(F_before);
                                operation_log["V_before"]["rows"] = V_before.rows();
                                operation_log["V_before"]["values"] = matrix_to_json(V_before);
                                operation_log["F_id_map_before"] = id_map_before;
                                operation_log["V_id_map_before"] = v_id_map_before;


                                // an after check here just to make sure this operation
                                // can be localy parametrized without problem
                                Eigen::VectorXd dbarea_before, dbarea_after;
                                igl::doublearea(V_before, F_before, dbarea_before);
                                igl::doublearea(V_after, F_after, dbarea_after);

                                if (dbarea_before.minCoeff() < 0) {
                                    std::cout << "dbarea_before:" << std::endl;
                                    std::cout << dbarea_before.transpose() << std::endl;
                                    visualize_meshes(V_before, F_before, V_after, F_after);
                                    throw std::runtime_error(
                                        operation_name + " negative area in F_before detected");
                                    scope.mark_failed();
                                    return {};
                                }

                                if (dbarea_after.minCoeff() < 0) {
                                    std::cout << "dbarea_after:" << std::endl;
                                    std::cout << dbarea_after.transpose() << std::endl;
                                    visualize_meshes(V_before, F_before, V_after, F_after);
                                    throw std::runtime_error(
                                        operation_name + " negative area in F_after detected");
                                    scope.mark_failed();
                                    return {};
                                }

                                if (wmtk::utils::hasSelfIntersection2D(V_before, F_before)) {
                                    throw std::runtime_error(
                                        operation_name + " self intersection in F_before detected");
                                    scope.mark_failed();
                                    return {};
                                }
                            } else {
                                operation_log["is_skipped"] = true;
                            }
                        }
                        // DEBUG:
                        if (false) {
                            auto [F_all, V_all, F_flag] =
                                static_cast<TriMesh&>(mesh()).get_FV_Fflag();
                            igl::writeOBJ(
                                OperationLogPath + "/VF_all_after_operation_" +
                                    std::to_string(succ_operations_count) + ".obj",
                                V_all,
                                F_all);
                            // write flag to a text file
                            std::ofstream flag_file(
                                OperationLogPath + "/F_flag_after_operation_" +
                                std::to_string(succ_operations_count) + ".txt");
                            if (flag_file.is_open()) {
                                for (int i = 0; i < F_flag.size(); ++i) {
                                    flag_file << F_flag[i] << " ";
                                }
                                flag_file.close();
                            } else {
                                std::cerr << "unable to open file for writing\n";
                            }
                        }

                    } else if (mesh().top_simplex_type() == PrimitiveType::Tetrahedron) {
                        // Prepare the mesh for the operation
                        bool is_simplex_boundary = mesh().parent_scope(
                            [&](const simplex::Simplex& s) { return mesh().is_boundary(s); },
                            simplex);

                        // TODO: what about swap operation
                        Eigen::MatrixXi T_after, T_before, F_bd_after, F_bd_before;
                        Eigen::MatrixXd V_after, V_before;

                        std::vector<int64_t> id_map_after, id_map_before, v_id_map_after,
                            v_id_map_before;

                        std::tie(T_after, V_after, F_bd_after, id_map_after, v_id_map_after) =
                            utils::get_local_tetmesh(
                                static_cast<const TetMesh&>(mesh()),
                                mods[0],
                                is_simplex_boundary && operation_name == "EdgeCollapse");

                        std::tie(T_before, V_before, F_bd_before, id_map_before, v_id_map_before) =
                            mesh().parent_scope(
                                [&](const simplex::Simplex& s) {
                                    if (operation_name == "EdgeCollapse")
                                        return utils::get_local_tetmesh_before_collapse(
                                            static_cast<const TetMesh&>(mesh()),
                                            s);
                                    return utils::get_local_tetmesh(
                                        static_cast<const TetMesh&>(mesh()),
                                        s,
                                        is_simplex_boundary && operation_name == "EdgeCollapse");
                                },
                                simplex);

                        // EdgeCollapse operation
                        if (operation_name == "EdgeCollapse") {
                            // check if there is a edge connected from interior to a boundary
                            // vertex
                            auto [is_bd_v0, is_bd_v1] = mesh().parent_scope(
                                [&](const simplex::Simplex& s) {
                                    return std::make_tuple(
                                        mesh().is_boundary(
                                            simplex::Simplex::vertex(mesh(), s.tuple())),
                                        mesh().is_boundary(simplex::Simplex::vertex(
                                            mesh(),
                                            mesh().switch_tuple(
                                                s.tuple(),
                                                PrimitiveType::Vertex))));
                                },
                                simplex);


                            if (is_simplex_boundary) {
                                // TODO: Figure out which vertex is collapse towards which vertex
                                // For now, let's assume it's simplex -> simplex.switch_vertex()
                                // re-obtain all the local mesh
                                // step1: get all the tet connect to vertex(simplex) before collapse
                                auto all_tets_before = mesh().parent_scope(
                                    [&](const simplex::Simplex& s) {
                                        return simplex::top_dimension_cofaces(
                                                   mesh(),
                                                   simplex::Simplex::vertex(mesh(), s.tuple()))
                                            .simplex_vector(PrimitiveType::Tetrahedron);
                                    },
                                    simplex);

                                // step2: get all_tets_after, which is is all tets in
                                // all_tets_before which is still valid
                                std::vector<simplex::Simplex> all_tets_after;
                                for (const auto& tet : all_tets_before) {
                                    if (mesh().is_valid(tet)) {
                                        all_tets_after.push_back(tet);
                                    }
                                }

                                // DEBUG: print out size of all_tets_before and all_tets_after
                                std::cout << "size of all_tets_before: " << all_tets_before.size()
                                          << std::endl;
                                std::cout << "size of all_tets_after: " << all_tets_after.size()
                                          << std::endl;

                                // step3: build TV_before and TV_after based on these tets
                                std::tie(T_before, V_before, id_map_before, v_id_map_before) =
                                    mesh().parent_scope(
                                        [&](const simplex::Simplex& s) {
                                            return utils::build_local_TV_matrix(
                                                static_cast<const TetMesh&>(mesh()),
                                                all_tets_before,
                                                simplex::Simplex::vertex(mesh(), s.tuple()));
                                        },
                                        simplex);
                                std::tie(T_after, V_after, id_map_after, v_id_map_after) =
                                    utils::build_local_TV_matrix(
                                        static_cast<const TetMesh&>(mesh()),
                                        all_tets_after,
                                        simplex::Simplex::vertex(mesh(), mods[0].tuple()));

                                if (T_after.rows() == 0) {
                                    std::cout << "T_after is empty" << std::endl;
                                    scope.mark_failed();
                                    return {};
                                }
                                std::cout << "v_id_map_before:" << std::endl;
                                for (int id = 0; id < v_id_map_before.size(); id++) {
                                    std::cout << "id: " << id << " : " << v_id_map_before[id]
                                              << std::endl;
                                }

                                std::cout << "v_id_map_after:" << std::endl;
                                for (int id = 0; id < v_id_map_after.size(); id++) {
                                    std::cout << "id: " << id << " : " << v_id_map_after[id]
                                              << std::endl;
                                }

                                // DEBUG: visualize the mesh before and after embedding
                                // utils::visualize_tet_mesh(V_before, T_before);
                                // utils::visualize_tet_mesh(V_after, T_after);
                                std::cout << "T_before:" << std::endl;
                                std::cout << T_before << std::endl;

                                std::cout << "T_after:" << std::endl;
                                std::cout << T_after << std::endl;

                                // TODO: find v1, so v0-v1 is the edge to collapse in before
                                int v1 = std::find(
                                             v_id_map_before.begin(),
                                             v_id_map_before.end(),
                                             v_id_map_after[0]) -
                                         v_id_map_before.begin();
                                if (v1 == 0 || v1 == v_id_map_before.size()) {
                                    throw std::runtime_error(
                                        "can't find v1, which is the edge to collapse in before");
                                }
                                std::cout << "edge to collapse in before: " << 0 << " " << v1
                                          << std::endl;
                                // embed the mesh!!!
                                auto V_before_param =
                                    utils::embed_mesh_lift(T_before, V_before, 0, v1);

                                if (V_before_param.rows() == 0) {
                                    scope.mark_failed();
                                    return {};
                                } else {
                                    Eigen::MatrixXd V_after_param(V_after.rows(), V_after.cols());
                                    int element_in_before_not_after = -1;
                                    for (int i = 0; i < v_id_map_before.size(); i++) {
                                        if (std::find(
                                                v_id_map_after.begin(),
                                                v_id_map_after.end(),
                                                v_id_map_before[i]) == v_id_map_after.end()) {
                                            element_in_before_not_after = i;
                                            break;
                                        }
                                    }


                                    // v0
                                    if (v_id_map_before[0] == v_id_map_after[0]) {
                                        V_after_param.row(0) =
                                            V_before_param.row(element_in_before_not_after);
                                    } else {
                                        auto it = std::find(
                                            v_id_map_before.begin(),
                                            v_id_map_before.end(),
                                            v_id_map_after[0]);
                                        if (it != v_id_map_before.end()) {
                                            int index = std::distance(v_id_map_before.begin(), it);
                                            V_after_param.row(0) = V_before_param.row(index);
                                        }
                                    }
                                    // other vertices
                                    for (int i = 1; i < v_id_map_after.size(); i++) {
                                        auto it = std::find(
                                            v_id_map_before.begin(),
                                            v_id_map_before.end(),
                                            v_id_map_after[i]);
                                        if (it != v_id_map_before.end()) {
                                            int index = std::distance(v_id_map_before.begin(), it);
                                            V_after_param.row(i) = V_before_param.row(index);
                                        }
                                    }

                                    // update the mesh to store in json
                                    V_before = V_before_param;
                                    V_after = V_after_param;
                                    // utils::visualize_tet_mesh(V_after_param, T_after);
                                } // end if (embedding is successful)

                            } // end if (is_simplex_boundary)
                        } // end if (operation_name == "EdgeCollapse")
                        // operation_log["success"] = true;
                        // STORE information to logfile
                        operation_log["T_after"]["rows"] = T_after.rows();
                        operation_log["T_after"]["values"] = matrix_to_json(T_after);
                        operation_log["V_after"]["rows"] = V_after.rows();
                        operation_log["V_after"]["values"] = matrix_to_json(V_after);
                        operation_log["F_bd_after"]["rows"] = F_bd_after.rows();
                        operation_log["F_bd_after"]["values"] = matrix_to_json(F_bd_after);
                        operation_log["T_id_map_after"] = id_map_after;
                        operation_log["V_id_map_after"] = v_id_map_after;

                        operation_log["T_before"]["rows"] = T_before.rows();
                        operation_log["T_before"]["values"] = matrix_to_json(T_before);
                        operation_log["V_before"]["rows"] = V_before.rows();
                        operation_log["V_before"]["values"] = matrix_to_json(V_before);
                        operation_log["F_bd_before"]["rows"] = F_bd_before.rows();
                        operation_log["F_bd_before"]["values"] = matrix_to_json(F_bd_before);
                        operation_log["T_id_map_before"] = id_map_before;
                        operation_log["V_id_map_before"] = v_id_map_before;

                        // operation_log["T_before_all"]["rows"] = T_before_all.rows();
                        // operation_log["T_before_all"]["values"] = matrix_to_json(T_before_all);
                        // operation_log["V_before_all"]["rows"] = V_before_all.rows();
                        // operation_log["V_before_all"]["values"] = matrix_to_json(V_before_all);
                        // operation_log["T_after_all"]["rows"] = T_after_all.rows();
                        // operation_log["T_after_all"]["values"] = matrix_to_json(T_after_all);
                    }

                    // TODO: get a larger json file to do this:
                    // op_logs_js["op_log"].push_back(operation_log);
                    operation_log_file << operation_log.dump(4);
                    operation_log_file.close();
                } else {
                    std::cerr << "unable to open file " << filename << " for writing\n";
                }
                succ_operations_count++;

            } // end if (m_record)
#endif
            return mods; // scope destructor is called
        }
    }

    scope.mark_failed();
    return {}; // scope destructor is called
}

bool Operation::before(const simplex::Simplex& simplex) const
{
#if defined(WMTK_ENABLE_HASH_UPDATE)
    const attribute::Accessor<int64_t> accessor = hash_accessor();

    if (!mesh().is_valid(simplex.tuple())) {
        return false;
    }
#else

    if (mesh().is_removed(simplex.tuple()) || !mesh().is_valid(simplex.tuple()) ||
        !mesh().is_valid(simplex)) {
        return false;
    }
#endif

    const auto simplex_resurrect = simplex;

    // map simplex to the invariant mesh
    const Mesh& invariant_mesh = m_invariants.mesh();

    if (&invariant_mesh == &mesh()) {
        if (!m_invariants.before(simplex_resurrect)) {
            return false;
        }
    } else {
        // TODO check if this is correct
        const std::vector<simplex::Simplex> invariant_simplices =
            m_mesh.map(invariant_mesh, simplex_resurrect);

        for (const simplex::Simplex& s : invariant_simplices) {
            if (!m_invariants.before(s)) {
                return false;
            }
        }
    }


    return true;
}

bool Operation::after(
    const std::vector<simplex::Simplex>& unmods,
    const std::vector<simplex::Simplex>& mods) const
{
    return m_invariants.directly_modified_after(unmods, mods);
}

void Operation::apply_attribute_transfer(const std::vector<simplex::Simplex>& direct_mods)
{
    if (m_attr_transfer_strategies.size() == 0) {
        return;
    }

    simplex::IdSimplexCollection all(m_mesh);
    all.reserve(100);


    for (const auto& s : direct_mods) {
        if (!s.tuple().is_null()) {
            for (const simplex::IdSimplex& ss : simplex::closed_star_iterable(m_mesh, s)) {
                all.add(ss);
            }
        }
    }
    if (direct_mods.size() > 1) {
        all.sort_and_clean();
    }

    for (const auto& at_ptr : m_attr_transfer_strategies) {
        if (&m_mesh == &(at_ptr->mesh())) {
            for (const simplex::IdSimplex& s : all.simplex_vector()) {
                if (s.primitive_type() == at_ptr->primitive_type()) {
                    at_ptr->run(m_mesh.get_simplex(s));
                }
            }
        } else {
            auto& at_mesh = at_ptr->mesh();
            auto at_mesh_simplices = m_mesh.map(at_mesh, direct_mods);

            simplex::IdSimplexCollection at_mesh_all(at_mesh);
            for (const simplex::Simplex& s : at_mesh_simplices) {
                for (const simplex::IdSimplex& ss : simplex::closed_star_iterable(at_mesh, s)) {
                    at_mesh_all.add(ss);
                }
            }

            at_mesh_all.sort_and_clean();

            for (const simplex::IdSimplex& s : at_mesh_all.simplex_vector()) {
                if (s.primitive_type() == at_ptr->primitive_type()) {
                    at_ptr->run(at_mesh.get_simplex(s));
                }
            }
        }
    }
}


void Operation::reserve_enough_simplices()
{
    // by default assume we'll at most create N * capacity
    constexpr static int64_t default_preallocation_size = 3;

    auto run = [&](auto&& m) {
        if constexpr (!std::is_const_v<std::remove_reference_t<decltype(m)>>) {
            auto cap = m.m_attribute_manager.m_capacities;
            for (auto& v : cap) {
                v *= default_preallocation_size;
            }
            m.reserve_more_attributes(cap);
        }
    };
    multimesh::MultiMeshVisitor visitor(run);
    visitor.execute_from_root(mesh());
}

} // namespace wmtk::operations
