#include "read_image_msh.hpp"

#include <igl/predicates/predicates.h>
#include <set>
#include <wmtk/Types.hpp>
#include <wmtk/components/simwild/EmbedSurface.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/io.hpp>

namespace wmtk::components::simwild {

Matrix4d get_input_transform(const nlohmann::json& json_params, const size_t input_index)
{
    const nlohmann::json& its_j = json_params["input_transform"];

    if (input_index >= its_j.size()) {
        return Matrix4d::Identity();
    }
    if (its_j[input_index].size() == 0) {
        return Matrix4d::Identity();
    }
    if (its_j[input_index].size() != 4) {
        log_and_throw_error("Input transform for input {} is invalid.", input_index);
    }

    const std::array<std::array<double, 4>, 4> it_arr = its_j[input_index];
    Matrix4d A;
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            A(i, j) = it_arr[i][j];
        }
    }
    logger().info("Input transform for input {}:\n{}", input_index, A);

    return A;
}

void positive_orientation_3D(const MatrixXd& V, MatrixXi& T)
{
    const Vector3d p0 = V.row(T(0, 0));
    const Vector3d p1 = V.row(T(0, 1));
    const Vector3d p2 = V.row(T(0, 2));
    const Vector3d p3 = V.row(T(0, 3));

    igl::predicates::exactinit();
    auto res = igl::predicates::orient3d(p0, p1, p2, p3);
    int result;
    if (res == igl::predicates::Orientation::POSITIVE)
        result = 1;
    else if (res == igl::predicates::Orientation::NEGATIVE)
        result = -1;
    else {
        log_and_throw_error(
            "First tet is degenerate! Vertices: \n{},\n{},\n{},\n{}",
            p0,
            p1,
            p2,
            p3);
    }

    bool is_inverted = result >= 0;
    if (is_inverted) {
        logger().warn("First tet of input is inverted -> invert all tets.");
        T.col(2).swap(T.col(3));
    }
}

void positive_orientation_2D(const MatrixXd& V, MatrixXi& F)
{
    const Vector2d p0 = V.row(F(0, 0));
    const Vector2d p1 = V.row(F(0, 1));
    const Vector2d p2 = V.row(F(0, 2));

    igl::predicates::exactinit();
    auto res = igl::predicates::orient2d(p0, p1, p2);
    int result;
    if (res == igl::predicates::Orientation::POSITIVE)
        result = 1;
    else if (res == igl::predicates::Orientation::NEGATIVE)
        result = -1;
    else {
        log_and_throw_error("First face is degenerate! Vertices: \n{},\n{},\n{}", p0, p1, p2);
    }

    bool is_inverted = result <= 0;
    if (is_inverted) {
        logger().warn("First face of input is inverted -> invert all faces.");
        F.col(1).swap(F.col(2));
    }
}

InputData read_image_msh(const std::string& path)
{
    InputData input_data;

    MshData msh;
    if (!std::filesystem::exists(path)) {
        log_and_throw_error("File {} does not exist.", path);
    }
    msh.load(path);

    std::optional<mshio::PhysicalGroup> ph_vol = msh.get_physical_group_by_name("ImageVolume");
    std::optional<mshio::PhysicalGroup> ph_env = msh.get_physical_group_by_name("EnvelopeSurface");

    const bool has_tets = msh.get_num_tets() != 0;
    const bool has_faces = msh.get_num_faces() != 0;

    const auto ph_groups = msh.get_physical_groups();

    if (!ph_groups.empty() && !(ph_vol && ph_env)) {
        logger().info("Reading input from pyhsical groups.");

        std::vector<MatrixXd> Vs;
        std::vector<MatrixXi> Fs;
        std::vector<std::string> tag_names;

        MatrixXd V_envelope;
        MatrixXi F_envelope;
        for (const mshio::PhysicalGroup& ph : msh.get_physical_groups()) {
            MatrixXd V;
            MatrixXi F;
            msh.get_VF(ph, V, F);
            tag_names.push_back(ph.name);
            if ((has_tets && ph.dim == 2) || (ph.dim == 1)) {
                // this must be the envelope surface
                if (V_envelope.size() != 0) {
                    log_and_throw_error("Multiple `envelope` groups found in {}", path);
                }
                input_data.V_envelope = V;
                input_data.F_envelope = F;
            } else {
                Vs.push_back(V);
                Fs.push_back(F);
            }
        }

        // combine all vertices
        if (Vs.empty()) {
            log_and_throw_error("No vertices found in {}", path);
        }
        auto& V = input_data.V_input;
        for (const auto& VV : Vs) {
            if (VV.rows() == 0) {
                continue;
            }
            const size_t n = V.rows();
            V.conservativeResize(n + VV.rows(), VV.cols());
            V.block(n, 0, VV.rows(), V.cols()) = VV;
        }

        // combine all tets
        if (has_tets) {
            if (Fs.empty()) {
                log_and_throw_error("No tets found in {}", path);
            }
            std::map<simplex::Tet, size_t> tet_ids;
            std::vector<Vector4i> tets;

            for (const auto& TT : Fs) {
                for (size_t i = 0; i < TT.rows(); ++i) {
                    const Vector4i& t = TT.row(i);
                    const simplex::Tet s(t[0], t[1], t[2], t[3]);
                    if (tet_ids.count(s) == 0) {
                        tet_ids[s] = tets.size();
                        tets.push_back(t);
                    }
                }
            }

            auto& T = input_data.T_input;
            T.resize(tets.size(), 4);
            for (size_t i = 0; i < tets.size(); ++i) {
                T.row(i) = tets[i];
            }

            // tags
            input_data.T_input_tag.resize(input_data.T_input.rows(), Fs.size() - 1);
            // assume Fs[0] are ambient faces
            for (size_t i = 1; i < Fs.size(); ++i) {
                for (size_t j = 0; j < Fs[i].rows(); ++j) {
                    const Vector4i& t = Fs[i].row(j);
                    const simplex::Tet s(t[0], t[1], t[2], t[3]);
                    const size_t tid = tet_ids[s];
                    input_data.T_input_tag.coeffRef(tid, i - 1) = 1;
                }
                input_data.tag_names.push_back(tag_names[i]);
            }
        } else {
            auto& Vi = input_data.V_input;
            Vi = Vi.block(0, 0, Vi.rows(), 2).eval();
            auto& Ve = input_data.V_envelope;
            if (Ve.size() > 0) {
                Ve = Ve.block(0, 0, Ve.rows(), 2).eval();
            }

            if (Fs.empty()) {
                log_and_throw_error("No faces found in {}", path);
            }
            std::map<simplex::Face, size_t> tet_ids;
            std::vector<Vector3i> tets;

            for (const auto& TT : Fs) {
                for (size_t i = 0; i < TT.rows(); ++i) {
                    const Vector3i& t = TT.row(i);
                    const simplex::Face s(t[0], t[1], t[2]);
                    if (tet_ids.count(s) == 0) {
                        tet_ids[s] = tets.size();
                        tets.push_back(t);
                    }
                }
            }

            auto& T = input_data.T_input;
            T.resize(tets.size(), 3);
            for (size_t i = 0; i < tets.size(); ++i) {
                T.row(i) = tets[i];
            }

            // tags
            input_data.T_input_tag.resize(input_data.T_input.rows(), Fs.size() - 1);
            // assume Fs[0] are ambient faces
            for (size_t i = 1; i < Fs.size(); ++i) {
                for (size_t j = 0; j < Fs[i].rows(); ++j) {
                    const Vector3i& t = Fs[i].row(j);
                    const simplex::Face s(t[0], t[1], t[2]);
                    const size_t tid = tet_ids[s];
                    input_data.T_input_tag.coeffRef(tid, i - 1) = 1;
                }
                input_data.tag_names.push_back(tag_names[i]);
            }
        }

        return input_data;
    }

    if (ph_vol && ph_env) {
        logger().info("Found ImageVolume and EnvelopeSurface.");

        if (ph_vol.value().dim != 3 && ph_vol.value().dim != 2) {
            log_and_throw_error(
                "Unexpected dimension {} of pysical group {}",
                ph_vol.value().dim,
                ph_vol.value().name);
        }
        if (ph_env.value().dim != 2 && ph_env.value().dim != 1) {
            log_and_throw_error(
                "Unexpected dimension {} of pysical group {}",
                ph_env.value().dim,
                ph_env.value().name);
        }

        if (has_faces && !has_tets) {
            logger().info("Read 2D input.");
            msh.get_VF(2, ph_vol.value().tag, input_data.V_input, input_data.T_input);
            msh.get_VF(1, ph_env.value().tag, input_data.V_envelope, input_data.F_envelope);
            auto& Vi = input_data.V_input;
            Vi = Vi.block(0, 0, Vi.rows(), 2).eval();
            auto& Ve = input_data.V_envelope;
            Ve = Ve.block(0, 0, Ve.rows(), 2).eval();
        } else {
            logger().info("Read 3D input.");
            msh.get_VF(3, ph_vol.value().tag, input_data.V_input, input_data.T_input);
            msh.get_VF(2, ph_env.value().tag, input_data.V_envelope, input_data.F_envelope);
        }

    } else {
        // only read volume directly from .msh and ignore other entities
        logger().info( //
            "Could not find pysical groups ImageVolume and EnvelopeSurface. Reading only "
            "tet data (in 2D face data) from MSH.");

        if (has_faces && !has_tets) {
            logger().info("Read 2D input.");
            input_data.V_input.resize(msh.get_num_face_vertices(), 2);
            input_data.T_input.resize(msh.get_num_faces(), 3);
            msh.extract_face_vertices([&input_data](size_t i, double x, double y, double z) {
                input_data.V_input.row(i) << x, y;
                if (z != 0) {
                    logger().warn("Ignoring non-zero z component of MSH for 2D input. z = {}", z);
                }
            });
            msh.extract_faces([&input_data](size_t i, size_t v0, size_t v1, size_t v2) {
                input_data.T_input.row(i) << (int)v0, (int)v1, (int)v2;
            });

            positive_orientation_2D(input_data.V_input, input_data.T_input);
        } else if (has_tets) {
            logger().info("Read 3D input.");
            input_data.V_input.resize(msh.get_num_tet_vertices(), 3);
            input_data.T_input.resize(msh.get_num_tets(), 4);
            msh.extract_tet_vertices([&input_data](size_t i, double x, double y, double z) {
                input_data.V_input.row(i) << x, y, z;
            });
            msh.extract_tets([&input_data](size_t i, size_t v0, size_t v1, size_t v2, size_t v3) {
                input_data.T_input.row(i) << (int)v0, (int)v1, (int)v2, (int)v3;
            });

            positive_orientation_3D(input_data.V_input, input_data.T_input);
        } else {
            log_and_throw_error("Input has neither 2D nor 3D input");
        }
    }

    if (input_data.T_input.cols() == 4) {
        log_and_throw_error("This code was not used for a long time and is probably outdated");
        int tets_tags_count = 0;
        for (const std::string& attr_name : msh.get_tet_attribute_names()) {
            if (attr_name.substr(0, 4) == "tag_") {
                ++tets_tags_count;
            }
        }

        input_data.T_input_tag.resize(input_data.T_input.rows(), tets_tags_count);
        for (const std::string& attr_name : msh.get_tet_attribute_names()) {
            if (attr_name.substr(0, 4) != "tag_") {
                continue;
            }
            const int tag_id = std::stoi(attr_name.substr(4));
            msh.extract_tet_attribute(
                attr_name,
                [&input_data, &tag_id](size_t i, std::vector<double> val) {
                    assert(val.size() == 1);
                    input_data.T_input_tag.coeffRef(i, tag_id) = val[0];
                });
        }
    } else {
        assert(input_data.T_input.cols() == 3);
        int tets_tags_count = 0;
        for (const std::string& attr_name : msh.get_all_element_attribute_names()) {
            if (attr_name.substr(0, 4) == "tag_") {
                ++tets_tags_count;
            }
        }
        MatrixXi input_tags;
        input_tags.resize(input_data.T_input.rows(), tets_tags_count);
        for (const std::string& attr_name : msh.get_all_element_attribute_names()) {
            if (attr_name.substr(0, 4) != "tag_") {
                continue;
            }
            const int tag_id = std::stoi(attr_name.substr(4));
            msh.extract_face_attribute(
                attr_name,
                [&tag_id, &input_tags](size_t i, std::vector<double> val) {
                    assert(val.size() == 1);
                    input_tags(i, tag_id) = val[0];
                });
        }

        // convert input tags to "binary"
        MatrixXi input_tags_bin;
        for (int j = 0; j < input_tags.cols(); ++j) {
            VectorXi img = input_tags.col(j);
            int max_id = img.maxCoeff();
            MatrixXi img_bin;
            img_bin.resize(img.rows(), max_id);
            img_bin.setZero();
            for (int i = 0; i < img.size(); ++i) {
                if (img[i] == 0) {
                    continue;
                }
                img_bin(i, img[i] - 1) = 1;
            }
            // input_data.T_input_tag.conservativeResize()
            auto& Ti = input_tags_bin;
            int nc = Ti.cols();
            Ti.conservativeResize(img_bin.rows(), Ti.cols() + img_bin.cols());
            Ti.block(0, nc, img_bin.rows(), img_bin.cols()) = img_bin;
        }

        input_data.T_input_tag = input_tags_bin.sparseView();
        for (int j = 0; j < input_tags_bin.cols(); ++j) {
            input_data.tag_names.push_back("tag_" + std::to_string(j));
        }
    }

    return input_data;
}

InputData read_mesh(
    const std::vector<std::string>& input_paths,
    const std::string& output_filename,
    const nlohmann::json& json_params)
{
    InputData input_data;

    std::vector<Matrix4d> input_transforms(input_paths.size());
    for (size_t i = 0; i < input_transforms.size(); ++i) {
        input_transforms[i] = get_input_transform(json_params, i);
    }

    const int NUM_THREADS = json_params["num_threads"];
    const bool debug_output = json_params["DEBUG_output"];
    const bool perform_sanity_checks = json_params["DEBUG_sanity_checks"];
    const bool preserve_topology = json_params["preserve_topology"];
    const bool skip_simplify = json_params["skip_simplify"];
    const double epsr_simplify = json_params["eps_simplify_rel"];
    double eps_simplify = json_params["eps_simplify"];
    const std::vector<std::string> input_names = json_params["input_names"];

    double tol_rel = -1;
    double tol_abs = -1;
    if (!preserve_topology) {
        tol_rel = 0.1 * epsr_simplify;
        tol_abs = 0.1 * eps_simplify;
    }

    // convert mesh into tet mesh
    EmbedSurface image_mesh(input_paths, input_transforms, tol_rel, tol_abs);
    image_mesh.m_perform_sanity_checks = perform_sanity_checks;

    if (debug_output) {
        image_mesh.write_surf_off(output_filename + "_input.off");
    }

    if (!preserve_topology && !skip_simplify) {
        /**
         * Simplify the input surface only if topology preservation is not required, since
         * simplification can change the topology. If topology preservation is required,
         * simplification is performed after insertion.
         */
        auto [bbox_min, bbox_max] = image_mesh.bbox_surf_minmax();
        double diag = (bbox_max - bbox_min).norm();
        if (epsr_simplify > 0) {
            eps_simplify = diag * epsr_simplify;
        }
        logger().info("Simplify before insertion with eps = {:.4}", eps_simplify);

        image_mesh.simplify_surface(eps_simplify, NUM_THREADS);

        if (debug_output) {
            image_mesh.write_surf_off(output_filename + "_input_simplified.off");
        }
    }

    input_data.V_envelope = image_mesh.V_surface();
    input_data.F_envelope = image_mesh.F_surface();

    const bool all_rounded = image_mesh.embed_surface(preserve_topology);
    image_mesh.consolidate();

    if (debug_output) {
        image_mesh.write_emb_surf_off(output_filename + "_input_emb.off");
    }

    input_data.V_input = image_mesh.V_emb();
    if (!all_rounded) {
        input_data.V_input_r = image_mesh.V_emb_r();
    }
    input_data.T_input = image_mesh.T_emb();
    input_data.T_input_tag = image_mesh.T_tags();
    // create generic tag names first
    for (int i = 0; i < input_data.T_input_tag.cols(); ++i) {
        input_data.tag_names.push_back("tag_" + std::to_string(i));
    }
    // use provided tag names if possible
    for (int i = 0; i < input_data.T_input_tag.cols(); ++i) {
        if (i < input_names.size()) {
            input_data.tag_names[i] = input_names[i];
        }
    }

    wmtk::logger().info("======= finish image-tet conversion =========");

    return input_data;
}


} // namespace wmtk::components::simwild