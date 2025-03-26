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
// helper function for tet edge collapse
std::vector<int> embed_mesh(const Eigen::MatrixXi& T, const Eigen::MatrixXi& F_bd)
{
    // Convert T and F_bd to sets for easier operations
    std::unordered_set<int> T_vertices;
    std::unordered_set<int> F_bd_vertices;

    // Collect all vertices from tetrahedra
    for (int i = 0; i < T.rows(); ++i) {
        for (int j = 0; j < T.cols(); ++j) {
            T_vertices.insert(T(i, j));
        }
    }

    // Collect all vertices from boundary faces
    for (int i = 0; i < F_bd.rows(); ++i) {
        for (int j = 0; j < F_bd.cols(); ++j) {
            F_bd_vertices.insert(F_bd(i, j));
        }
    }

    // Find vertices that are in T but not in F_bd
    std::unordered_set<int> non_bd_vertices;
    for (const auto& v : T_vertices) {
        if (F_bd_vertices.find(v) == F_bd_vertices.end()) {
            non_bd_vertices.insert(v);
        }
    }

    // Initialize detailed statistics dictionary
    std::map<std::string, int> tet_stats = {
        {"bd_0", 0}, // 0 boundary vertices
        {"bd_1", 0}, // 1 boundary vertex
        {"bd_2", 0}, // 2 boundary vertices
        {"bd_3", 0}, // 3 boundary vertices
        {"bd_4", 0} // 4 boundary vertices
    };

    // Iterate through each tetrahedron
    for (int i = 0; i < T.rows(); ++i) {
        // Count boundary vertices
        int bd_count = 0;
        for (int j = 0; j < T.cols(); ++j) {
            if (F_bd_vertices.find(T(i, j)) != F_bd_vertices.end()) {
                bd_count++;
            }
        }
        // Classify based on boundary vertex count
        tet_stats["bd_" + std::to_string(bd_count)]++;
    }

    // Print detailed statistics
    spdlog::info("Tetrahedron Statistics:");
    spdlog::info("Number of tetrahedra with 0 boundary vertices: {}", tet_stats["bd_0"]);
    spdlog::info("Number of tetrahedra with 1 boundary vertex: {}", tet_stats["bd_1"]);
    spdlog::info("Number of tetrahedra with 2 boundary vertices: {}", tet_stats["bd_2"]);
    spdlog::info("Number of tetrahedra with 3 boundary vertices: {}", tet_stats["bd_3"]);
    spdlog::info("Number of tetrahedra with 4 boundary vertices: {}", tet_stats["bd_4"]);

    // Create a dictionary to store the count of boundary neighbors for each non-boundary vertex
    std::unordered_map<int, int> bd_neighbor_counts;
    for (const auto& v : non_bd_vertices) {
        bd_neighbor_counts[v] = 0;
    }

    // Create sets to store boundary neighbors for each non-boundary vertex
    std::unordered_map<int, std::unordered_set<int>> bd_neighbor_sets;
    for (const auto& v : non_bd_vertices) {
        bd_neighbor_sets[v] = std::unordered_set<int>();
    }

    // For each tetrahedron
    for (int i = 0; i < T.rows(); ++i) {
        // For each vertex in the tetrahedron
        for (int j = 0; j < T.cols(); ++j) {
            int v = T(i, j);
            // If this vertex is non-boundary
            if (non_bd_vertices.find(v) != non_bd_vertices.end()) {
                // Add boundary neighbors from this tetrahedron to the set
                for (int k = 0; k < T.cols(); ++k) {
                    int neighbor = T(i, k);
                    if (F_bd_vertices.find(neighbor) != F_bd_vertices.end() && neighbor != v) {
                        bd_neighbor_sets[v].insert(neighbor);
                    }
                }
            }
        }
    }

    // Calculate the number of unique boundary neighbors for each non-boundary vertex
    for (const auto& v : non_bd_vertices) {
        bd_neighbor_counts[v] = bd_neighbor_sets[v].size();
    }

    // Print boundary neighbor counts for each non-boundary vertex
    spdlog::info("Boundary neighbor counts for non-boundary vertices:");
    for (const auto& [vertex, count] : bd_neighbor_counts) {
        spdlog::info("Vertex {} has {} boundary neighbors", vertex, count);
    }

    // Convert non-boundary vertices set to vector and return
    std::vector<int> result(non_bd_vertices.begin(), non_bd_vertices.end());
    return result;
}

Eigen::MatrixXi find_F_top(const std::unordered_set<int>& non_bd_vertices, const Eigen::MatrixXi& T)
{
    std::set<std::array<int, 3>> F_top_set;

    // For each tetrahedron
    for (int i = 0; i < T.rows(); ++i) {
        // Get all possible triangular faces from the tetrahedron
        std::array<std::array<int, 3>, 4> faces = {
            std::array<int, 3>{T(i, 0), T(i, 1), T(i, 2)},
            std::array<int, 3>{T(i, 0), T(i, 1), T(i, 3)},
            std::array<int, 3>{T(i, 0), T(i, 2), T(i, 3)},
            std::array<int, 3>{T(i, 1), T(i, 2), T(i, 3)}};

        // Check each face
        for (auto face : faces) {
            // If all vertices of the face are interior vertices
            if (non_bd_vertices.find(face[0]) != non_bd_vertices.end() &&
                non_bd_vertices.find(face[1]) != non_bd_vertices.end() &&
                non_bd_vertices.find(face[2]) != non_bd_vertices.end()) {
                // Sort vertices to ensure consistent orientation
                std::sort(face.begin(), face.end());
                F_top_set.insert(face);
            }
        }
    }

    // Convert set to Eigen::MatrixXi
    Eigen::MatrixXi F_top(F_top_set.size(), 3);
    int row = 0;
    for (const auto& face : F_top_set) {
        F_top(row, 0) = face[0];
        F_top(row, 1) = face[1];
        F_top(row, 2) = face[2];
        row++;
    }

    return F_top;
}


std::vector<simplex::Simplex> Operation::operator()(const simplex::Simplex& simplex)
{
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
                        auto has_self_intersection = [&](const Eigen::MatrixXi& F,
                                                         const Eigen::MatrixXd& uv) -> bool {
                            // Extract the boundary loop
                            Eigen::VectorXi loop;
                            igl::boundary_loop(F, loop);

                            int n = loop.size();

                            // Helper function to check if two segments intersect
                            auto doIntersect = [](const Eigen::RowVector2d& p1,
                                                  const Eigen::RowVector2d& q1,
                                                  const Eigen::RowVector2d& p2,
                                                  const Eigen::RowVector2d& q2) -> bool {
                                auto onSegment = [](const Eigen::RowVector2d& p,
                                                    const Eigen::RowVector2d& q,
                                                    const Eigen::RowVector2d& r) -> bool {
                                    return q[0] <= std::max(p[0], r[0]) &&
                                           q[0] >= std::min(p[0], r[0]) &&
                                           q[1] <= std::max(p[1], r[1]) &&
                                           q[1] >= std::min(p[1], r[1]);
                                };

                                auto orientation = [](const Eigen::RowVector2d& p,
                                                      const Eigen::RowVector2d& q,
                                                      const Eigen::RowVector2d& r) -> int {
                                    double val = (q[1] - p[1]) * (r[0] - q[0]) -
                                                 (q[0] - p[0]) * (r[1] - q[1]);
                                    if (val == 0) return 0; // collinear
                                    return (val > 0) ? 1 : 2; // clock or counterclock wise
                                };

                                int o1 = orientation(p1, q1, p2);
                                int o2 = orientation(p1, q1, q2);
                                int o3 = orientation(p2, q2, p1);
                                int o4 = orientation(p2, q2, q1);

                                // General case
                                if (o1 != o2 && o3 != o4) return true;

                                // Special Cases
                                if (o1 == 0 && onSegment(p1, p2, q1)) return true;
                                if (o2 == 0 && onSegment(p1, q2, q1)) return true;
                                if (o3 == 0 && onSegment(p2, p1, q2)) return true;
                                if (o4 == 0 && onSegment(p2, q1, q2)) return true;

                                return false; // No intersection
                            };

                            for (int i = 0; i < n; ++i) {
                                Eigen::RowVector2d p1 = uv.row(loop[i]);
                                Eigen::RowVector2d q1 = uv.row(loop[(i + 1) % n]);
                                for (int j = i + 2; j < n; ++j) {
                                    if (j != (i + 1) % n &&
                                        i != (j + 1) % n) { // Skip adjacent edges
                                        Eigen::RowVector2d p2 = uv.row(loop[j]);
                                        Eigen::RowVector2d q2 = uv.row(loop[(j + 1) % n]);
                                        if (doIntersect(p1, q1, p2, q2)) {
                                            return true;
                                        }
                                    }
                                }
                            }

                            return false;
                        };
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

                            // TODO: add an after check here just to make sure this operation can be
                            // localy parametrized without problem
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

                            if (has_self_intersection(F_before, UV_joint)) {
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
                                    throw std::runtime_error(
                                        "swap/smooth negative area in F_before detected");
                                    scope.mark_failed();
                                    return {};
                                }

                                if (dbarea_after.minCoeff() < 0) {
                                    throw std::runtime_error(
                                        "swap/smooth negative area in F_after detected");
                                    scope.mark_failed();
                                    return {};
                                }


                                if (has_self_intersection(F_before, V_before)) {
                                    throw std::runtime_error(
                                        "swap/smooth self intersection in F_before detected");
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
                        bool is_simplex_boundary = mesh().parent_scope(
                            [&](const simplex::Simplex& s) { return mesh().is_boundary(s); },
                            simplex);
                        // TODO: what about swap operation?
                        bool need_get_boundary_surface =
                            is_simplex_boundary && operation_name == "EdgeCollapse";
                        auto [T_after, V_after, F_bd_after, id_map_after, v_id_map_after] =
                            utils::get_local_tetmesh(
                                static_cast<const TetMesh&>(mesh()),
                                mods[0],
                                is_simplex_boundary && operation_name == "EdgeCollapse");

                        auto [T_before, V_before, F_bd_before, id_map_before, v_id_map_before] =
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
                        if (operation_name == "EdgeCollapse") {
                            // check if there is a edge connected from interior to a boundary vertex
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
                            if (!is_simplex_boundary && (is_bd_v0 || is_bd_v1)) {
                                // Get vertex position after operation
                                Eigen::Vector3d after_vertex_position;
                                const auto& pos_accessor = mesh().get_attribute_handle<double>(
                                    "vertices",
                                    PrimitiveType::Vertex);
                                auto pos_accessor_vector =
                                    mesh().create_const_accessor<double>(pos_accessor);
                                after_vertex_position =
                                    pos_accessor_vector.const_vector_attribute(mods[0].tuple());
                                std::cout << "Vertex position after operation: "
                                          << after_vertex_position.transpose() << std::endl;

                                // Get boundary and non-boundary vertex positions in parent scope
                                Eigen::Vector3d boundary_vertex_position;
                                Eigen::Vector3d non_boundary_vertex_position;

                                auto get_vertex_positions = [&](const simplex::Simplex& s) {
                                    auto v0_tuple =
                                        simplex::Simplex::vertex(mesh(), s.tuple()).tuple();
                                    auto v1_tuple =
                                        simplex::Simplex::vertex(
                                            mesh(),
                                            mesh().switch_tuple(s.tuple(), PrimitiveType::Vertex))
                                            .tuple();

                                    const auto& parent_pos_accessor =
                                        mesh().get_attribute_handle<double>(
                                            "vertices",
                                            PrimitiveType::Vertex);
                                    auto parent_pos_accessor_vector =
                                        mesh().create_const_accessor<double>(parent_pos_accessor);

                                    Eigen::Vector3d v0_position =
                                        parent_pos_accessor_vector.const_vector_attribute(v0_tuple);
                                    Eigen::Vector3d v1_position =
                                        parent_pos_accessor_vector.const_vector_attribute(v1_tuple);

                                    if (is_bd_v0 && !is_bd_v1) {
                                        boundary_vertex_position = v0_position;
                                        non_boundary_vertex_position = v1_position;
                                    } else if (!is_bd_v0 && is_bd_v1) {
                                        boundary_vertex_position = v1_position;
                                        non_boundary_vertex_position = v0_position;
                                    } else if (is_bd_v0 && is_bd_v1) {
                                        // Case where both vertices are on boundary
                                        boundary_vertex_position = v0_position;
                                        non_boundary_vertex_position = v1_position;
                                    }
                                };

                                mesh().parent_scope(get_vertex_positions, simplex);

                                std::cout << "Boundary vertex position: "
                                          << boundary_vertex_position.transpose() << std::endl;
                                std::cout << "Non-boundary vertex position: "
                                          << non_boundary_vertex_position.transpose() << std::endl;

                                // Check if boundary vertex position matches position after
                                // operation
                                if ((boundary_vertex_position - after_vertex_position).norm() <
                                    1e-10) {
                                    std::cout << "Boundary vertex position matches position after "
                                                 "operation, allowing operation to continue"
                                              << std::endl;
                                    // Positions match, allow operation to continue
                                } else {
                                    std::cout << "Boundary vertex position does not match position "
                                                 "after operation, operation failed"
                                              << std::endl;

                                    // Check if non-boundary vertex position matches
                                    if ((non_boundary_vertex_position - after_vertex_position)
                                            .norm() < 1e-10) {
                                        std::cout << "Non-boundary vertex position matches "
                                                     "position after operation"
                                                  << std::endl;
                                    } else {
                                        std::cout << "Non-boundary vertex position does not match "
                                                     "position after operation"
                                                  << std::endl;
                                    }

                                    std::cerr << "boundary vertex in EdgeCollapse\n";
                                    // scope.mark_failed();
                                    // return {};
                                }
                            }

                            if (is_simplex_boundary) {
                                std::vector<int> non_bd_vertices =
                                    embed_mesh(T_before, F_bd_before);
                                Eigen::MatrixXi F_top = find_F_top(
                                    std::unordered_set<int>(
                                        non_bd_vertices.begin(),
                                        non_bd_vertices.end()),
                                    T_before);

                                // Use harmonic parameterization
                                Eigen::MatrixXd uv;
                                Eigen::VectorXi bnd;
                                Eigen::MatrixXd bnd_uv;

                                // First remove unreferenced vertices
                                Eigen::MatrixXd V_clean;
                                Eigen::MatrixXi F_clean;
                                Eigen::VectorXi IM, J;
                                igl::remove_unreferenced(
                                    V_before,
                                    F_bd_before,
                                    V_clean,
                                    F_clean,
                                    IM,
                                    J);

                                // Get boundary loop
                                igl::boundary_loop(F_clean, bnd);

                                // Map boundary to circle while preserving edge proportions
                                bnd_uv.resize(bnd.size(), 2);
                                for (int i = 0; i < bnd.size(); i++) {
                                    double angle = 2.0 * M_PI * i / bnd.size();
                                    bnd_uv(i, 0) = cos(angle);
                                    bnd_uv(i, 1) = sin(angle);
                                }

                                // Scale boundary to match area
                                Eigen::MatrixXd M_before;
                                igl::doublearea(V_clean, F_clean, M_before);
                                bnd_uv *= sqrt(M_before.sum() / (2 * 3.14159265358979323846));

                                // Function to check UV orientation
                                auto check_uv_orientation = [](const Eigen::MatrixXd& uv,
                                                               const Eigen::MatrixXi& F) {
                                    for (int i = 0; i < F.rows(); i++) {
                                        if (wmtk::utils::wmtk_orient2d(
                                                uv.row(F(i, 0)),
                                                uv.row(F(i, 1)),
                                                uv.row(F(i, 2))) < 0) {
                                            return false;
                                        }
                                    }
                                    return true;
                                };

                                // Compute harmonic parameterization
                                Eigen::MatrixXd clean_uv;
                                igl::harmonic(V_clean, F_clean, bnd, bnd_uv, 1, clean_uv);

                                // Map parameterization back to original mesh indices
                                uv.resize(V_before.rows(), 2);
                                for (int i = 0; i < IM.size(); i++) {
                                    uv.row(IM(i)) = clean_uv.row(i);
                                }

                                // Check for flipped triangles
                                if (!check_uv_orientation(clean_uv, F_clean)) {
                                    std::cerr
                                        << "Harmonic parameterization produced flipped triangles"
                                        << std::endl;
                                }

                                // Calculate parameterization quality metrics
                                Eigen::VectorXd areas;
                                igl::doublearea(clean_uv, F_clean, areas);
                                areas /= 2.0;
                                // Output UV parameterization information
                                double total_area = areas.sum();
                                double min_area = areas.minCoeff();
                                double max_area = areas.maxCoeff();

                                std::cout << "Parameterization info:" << std::endl;
                                std::cout << "  Total UV area: " << total_area << std::endl;
                                std::cout << "  Min triangle area: " << min_area << std::endl;
                                std::cout << "  Max triangle area: " << max_area << std::endl;

// Optionally visualize the UV map
#ifdef WMTK_DEBUG_UV
                                igl::opengl::glfw::Viewer viewer;
                                viewer.data().set_mesh(clean_uv, F_clean);
                                viewer.launch();
#endif
                            }
                        }
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

    if (!mesh().is_valid(simplex.tuple())) { // TODO: chang to is_removed and resurrect later
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
