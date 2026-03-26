#include "topological_offset.hpp"

#include <vector>

#include <jse/jse.h>
#include <wmtk/TetMesh.h>
#include <wmtk/Types.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/remove_unreferenced.h>
#include <igl/write_triangle_mesh.h>
#include "Parameters.h"
#include "TopoOffsetTetMesh.h"
#include "TopoOffsetTriMesh.h"
#include "read_image_msh.hpp"

#include "topological_offset_spec.hpp"


namespace wmtk::components::topological_offset {


void topological_offset(nlohmann::json json_params)
{
    using wmtk::utils::resolve_path;

    // verify input and inject defaults
    {
        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(json_params, topological_offset_spec);
        if (!r) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, topological_offset_spec);
    }

    const std::filesystem::path root = json_params["json_input_file"];

    // load input file path
    std::string input_path = resolve_path(root, json_params["input"]).string();

    // load params
    Parameters params;
    for (const std::vector<int>& offset_tag_pair : json_params["offset_tags"]) {
        params.offset_tags.push_back({{offset_tag_pair[0], offset_tag_pair[1]}});
    }
    for (const std::vector<int>& offset_tag_val_pair : json_params["offset_tag_val"]) {
        params.offset_tag_val.push_back({{offset_tag_val_pair[0], offset_tag_val_pair[1]}});
    }
    params.target_distance = json_params["target_distance"];
    params.relative_ball_threshold = json_params["relative_ball_threshold"];
    params.edge_search_term_len = json_params["edge_search_termination_len"];
    params.sorted_marching = json_params["sorted_marching"];
    if (params.relative_ball_threshold < 0.0 || params.relative_ball_threshold > 1.0) {
        log_and_throw_error(
            "Invalid relative_ball_threshold [{}], must be between 0 and 1.",
            params.relative_ball_threshold);
    }
    params.output_path = resolve_path(root, json_params["output"]).string();
    bool check_manifoldness = json_params["check_manifoldness"];

    std::filesystem::path output_filename = params.output_path;
    if (output_filename.has_extension() && output_filename.extension() != ".msh") {
        output_filename.replace_extension(".msh");
        logger().warn(
            "Extension of provided output filename is ignored. Output will be {}",
            output_filename.string());
    }
    output_filename.replace_extension(""); // extension is added back later

    // int NUM_THREADS = json_params["num_threads"];
    int NUM_THREADS = 0;
    params.save_vtu = json_params["save_vtu"];
    params.debug_output = json_params["DEBUG_output"];

    // input must be .msh
    if (std::filesystem::path(input_path).extension() != ".msh") {
        log_and_throw_error("Input must be a .msh file.");
    }

    MatrixXd V_input;
    MatrixXi F_input;
    MatrixXd F_input_tags;
    read_image_msh(input_path, V_input, F_input, F_input_tags);

    if (F_input.cols() == 3) { // input is a 2d tri mesh
        logger().info("Input mesh (2D trimesh): {}", input_path);

        // initialize mesh
        TopoOffsetTriMesh mesh(params, NUM_THREADS);
        mesh.init_from_image(V_input, F_input, F_input_tags);

        // check empty input
        if (mesh.empty_input_complex()) {
            logger().info("Empty input complex. Aborting.");
            return;
        }

        // check for inversions in input mesh
        auto tris_before = mesh.get_faces();
        if (!mesh.invariants(tris_before)) {
            std::string bad_tris_str = "";
            for (const TriMesh::Tuple& t : tris_before) {
                std::vector<TriMesh::Tuple> tvec;
                tvec.push_back(t);
                if (!mesh.invariants(tvec)) {
                    bad_tris_str += (std::to_string(t.fid(mesh)) + " ");
                }
            }
            log_and_throw_error("Inverted input element. Aborting. Bad tri ids: {}", bad_tris_str);
        }
        tris_before.clear();

        // initialize BVH
        mesh.init_input_complex_bvh();
        mesh.consolidate_mesh();

        // set initial counts
        mesh.m_init_counts[0] = mesh.get_vertices().size();
        mesh.m_init_counts[1] = mesh.get_edges().size();
        mesh.m_init_counts[2] = mesh.get_faces().size();

        // output input complex and entire mesh as vtu
        if (mesh.m_params.debug_output) {
            mesh.write_vtu(output_filename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
            mesh.write_input_complex(output_filename.string() + "_input_complex");
        }

        // start timer
        igl::Timer timer;
        timer.start();

        // make embedding simplicial
        logger().info("Creating simplicial embedding...");
        if (!mesh.is_simplicially_embedded()) {
            mesh.simplicial_embedding();
            bool dummy = mesh.is_simplicially_embedded();
        }
        mesh.consolidate_mesh();
        if (mesh.m_params.debug_output) {
            mesh.write_vtu(output_filename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
        }

        // perform offset
        logger().info("Performing offset...");
        if (mesh.m_params.target_distance <= 0.0) {
            mesh.marching_tets();
            mesh.set_offset_tri_tags();
            mesh.consolidate_mesh();
        } else { // conservative growth
            // run BFS, save after
            mesh.grow_offset_conservative();
            mesh.consolidate_mesh();
            if (mesh.m_params.debug_output) {
                mesh.set_offset_tri_tags();
                mesh.write_vtu(output_filename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
            }

            // simplicially embed again, if needed
            if (!mesh.is_simplicially_embedded()) {
                mesh.simplicial_embedding();
                bool dummy = mesh.is_simplicially_embedded();
                mesh.consolidate_mesh();
            }
            if (mesh.m_params.debug_output) {
                mesh.set_offset_tri_tags();
                mesh.write_vtu(output_filename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
            }

            // marching tets (using binary search edge split)
            mesh.m_edge_split_mode = TopoOffsetTriMesh::EdgeSplitMode::BinarySearch;
            mesh.marching_tets();
            mesh.m_edge_split_mode = TopoOffsetTriMesh::EdgeSplitMode::Midpoint;
            mesh.consolidate_mesh();
            mesh.set_offset_tri_tags();
        }

        // stop timer
        double time = timer.getElapsedTime();
        wmtk::logger().info("total time {}s", time);

        // inversion check
        auto tris = mesh.get_faces();
        bool noninverted = mesh.invariants(tris);
        if (!noninverted) {
            std::string bad_tris_str = "";
            for (const TriMesh::Tuple& t : tris) {
                std::vector<TriMesh::Tuple> tvec;
                tvec.push_back(t);
                if (!mesh.invariants(tvec)) {
                    bad_tris_str += (" " + std::to_string(t.fid(mesh)));
                }
            }
            mesh.write_msh(output_filename.string()); // DEBUG: write .msh anyway
            log_and_throw_error("INVERSION DURING OFFSET! bad tri ids: {}", bad_tris_str);
        }

        // offset region manifoldness check
        if (check_manifoldness) {
            if (mesh.offset_is_manifold()) {
                logger().info("Offset region manifold check passed.");
            } else {
                mesh.write_msh(output_filename.string()); // DEBUG: write .msh anyway
                log_and_throw_error("OFFSET REGION IS NOT MANIFOLD");
            }
        }

        // output
        std::ofstream fout(output_filename.string() + ".log");
        fout << "before:" << std::endl;
        fout << "\t#f: " << mesh.m_init_counts[2] << std::endl;
        fout << "\t#e: " << mesh.m_init_counts[1] << std::endl;
        fout << "\t#v: " << mesh.m_init_counts[0] << std::endl;
        fout << "after:" << std::endl;
        fout << "\t#f: " << mesh.get_faces().size() << std::endl;
        fout << "\t#e: " << mesh.get_edges().size() << std::endl;
        fout << "\t#v: " << mesh.get_vertices().size() << std::endl;
        fout << "threads: " << NUM_THREADS << std::endl;
        fout << "time: " << time << std::endl;
        fout.close();

        mesh.write_msh(output_filename.string()); // write .msh
        if (mesh.m_params.debug_output) {
            mesh.write_vtu(output_filename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
        } else if (mesh.m_params.save_vtu) { // write .vtu
            mesh.write_vtu(output_filename.string());
        }

        wmtk::logger().info("======= finish =========");
    } else { // input is a 3d tet mesh
        logger().info("Input mesh (3D tetmesh): {}", input_path);

        // initialize mesh
        TopoOffsetTetMesh mesh(params, NUM_THREADS);
        mesh.init_from_image(V_input, F_input, F_input_tags);

        // check empty input
        if (mesh.empty_input_complex()) {
            logger().info("Empty input complex. Aborting.");
            return;
        }

        // check for inversions in input mesh
        auto tets_before = mesh.get_tets();
        if (!mesh.invariants(tets_before)) {
            std::string bad_tets_str = "";
            for (const TetMesh::Tuple& t : tets_before) {
                std::vector<TetMesh::Tuple> tvec;
                tvec.push_back(t);
                if (!mesh.invariants(tvec)) {
                    bad_tets_str += (std::to_string(t.tid(mesh)) + " ");
                }
            }
            log_and_throw_error("Inverted input element. Aborting. Bad tet ids: {}", bad_tets_str);
        }
        tets_before.clear();

        // initialize BVH
        mesh.init_input_complex_bvh();
        mesh.consolidate_mesh();

        // record counts (mostly debugging, this is probably really slow)
        mesh.m_init_counts[0] = mesh.vertex_size();
        mesh.m_init_counts[1] = mesh.get_edges().size();
        mesh.m_init_counts[2] = mesh.get_faces().size();
        mesh.m_init_counts[3] = mesh.tet_size();

        // output input complex and entire mesh as vtu
        if (mesh.m_params.debug_output) {
            mesh.write_vtu(output_filename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
            mesh.write_input_complex(output_filename.string() + "_input_complex");
        }

        // start timer
        igl::Timer timer;
        timer.start();

        // make embedding simplicial (split components per Alg 1)
        logger().info("Creating simplicial embedding...");
        if (!mesh.is_simplicially_embedded()) { // internally prints to console
            mesh.simplicial_embedding();
            bool dummy = mesh.is_simplicially_embedded(); // internally prints to console
        }
        mesh.consolidate_mesh();
        if (mesh.m_params.debug_output) { // intermediate output
            mesh.write_vtu(output_filename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
        }

        // perform offset
        logger().info("Performing offset...");
        if (mesh.m_params.target_distance <= 0.0) { // midpoint split offset
            mesh.marching_tets();
            mesh.set_offset_tet_tags();
            mesh.consolidate_mesh();
        } else { // variable offset distance
            // run BFS, save after
            mesh.grow_offset_conservative();
            mesh.consolidate_mesh();
            if (mesh.m_params.debug_output) {
                mesh.set_offset_tet_tags();
                mesh.write_vtu(output_filename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
            }

            // simplicially embed again, if needed
            if (!mesh.is_simplicially_embedded()) {
                mesh.simplicial_embedding();
                bool dummy = mesh.is_simplicially_embedded();
                mesh.consolidate_mesh();
            }
            if (mesh.m_params.debug_output) {
                mesh.set_offset_tet_tags();
                mesh.write_vtu(output_filename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
            }

            // marching tets (using binary search edge split)
            mesh.m_edge_split_mode = TopoOffsetTetMesh::EdgeSplitMode::BinarySearch;
            mesh.marching_tets();
            mesh.m_edge_split_mode = TopoOffsetTetMesh::EdgeSplitMode::Midpoint;
            mesh.consolidate_mesh();
            mesh.set_offset_tet_tags();
        }

        // stop timer
        double time = timer.getElapsedTime();
        wmtk::logger().info("total time {}s", time);

        // inversion check
        auto tets = mesh.get_tets();
        bool noninverted = mesh.invariants(tets);
        if (!noninverted) {
            std::string bad_tets_str = "";
            for (const TetMesh::Tuple& t : tets) {
                std::vector<TetMesh::Tuple> tvec;
                tvec.push_back(t);
                if (!mesh.invariants(tvec)) {
                    bad_tets_str += (" " + std::to_string(t.tid(mesh)));
                }
            }
            mesh.write_msh(output_filename.string()); // DEBUG write .msh anyway
            log_and_throw_error("INVERSION DURING OFFSET! bad tet ids: {}", bad_tets_str);
        }

        // offset region manifoldness check
        if (check_manifoldness) {
            if (mesh.offset_is_manifold()) {
                logger().info("Offset region manifold check passed.");
            } else {
                mesh.write_msh(output_filename.string()); // DEBUG: write .msh anyway
                log_and_throw_error("OFFSET REGION IS NOT MANIFOLD");
            }
        }

        // output
        std::ofstream fout(output_filename.string() + ".log");
        fout << "before:" << std::endl;
        fout << "\t#t: " << mesh.m_init_counts[3] << std::endl;
        fout << "\t#f: " << mesh.m_init_counts[2] << std::endl;
        fout << "\t#e: " << mesh.m_init_counts[1] << std::endl;
        fout << "\t#v: " << mesh.m_init_counts[0] << std::endl;
        fout << "after:" << std::endl;
        fout << "\t#t: " << mesh.tet_size() << std::endl;
        fout << "\t#f: " << mesh.get_faces().size() << std::endl;
        fout << "\t#e: " << mesh.get_edges().size() << std::endl;
        fout << "\t#v: " << mesh.vertex_size() << std::endl;
        fout << "threads: " << NUM_THREADS << std::endl;
        fout << "time: " << time << std::endl;
        fout.close();

        mesh.write_msh(output_filename.string()); // write .msh
        if (mesh.m_params.debug_output) {
            mesh.write_vtu(output_filename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
        } else if (mesh.m_params.save_vtu) { // write .vtu
            mesh.write_vtu(output_filename.string());
        }

        wmtk::logger().info("======= finish =========");
    }
}


} // namespace wmtk::components::topological_offset