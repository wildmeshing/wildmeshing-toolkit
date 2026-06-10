#include "read_image_msh.hpp"
#include <filesystem>
#include <set>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/utils/io.hpp>


namespace wmtk::components::topological_offset {


void positive_orientation_2D(const MatrixXd& V, MatrixXi& F)
{
    const Vector2d p0 = V.row(F(0, 0));
    const Vector2d p1 = V.row(F(0, 1));
    const Vector2d p2 = V.row(F(0, 2));

    igl::predicates::exactinit();
    auto res = igl::predicates::orient2d(p0, p1, p2);
    int result;
    if (res == igl::predicates::Orientation::POSITIVE) {
        result = 1;
    } else if (res == igl::predicates::Orientation::NEGATIVE) {
        result = -1;
    } else {
        log_and_throw_error("First face is degenerate! Vertices: \n{},\n{},\n{}", p0, p1, p2);
    }

    bool is_inverted = (result <= 0);
    if (is_inverted) {
        logger().warn("First face of input is inverted -> invert all faces.");
        F.col(1).swap(F.col(2));
    }
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
    if (res == igl::predicates::Orientation::POSITIVE) {
        result = 1;
    } else if (res == igl::predicates::Orientation::NEGATIVE) {
        result = -1;
    } else {
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

    bool has_tets = msh.get_num_tets() != 0;
    bool has_faces = msh.get_num_faces() != 0;
    const auto& ph_groups = msh.get_physical_groups();

    if (!ph_groups.empty() && !(ph_vol && ph_env)) {
        logger().info("Reading input from physical groups.");

        std::vector<MatrixXd> Vs;
        std::vector<MatrixXi> Fs;
        std::vector<std::string> tag_names;
        MatrixXd V_env;
        MatrixXi F_env;

        for (const mshio::PhysicalGroup& group : msh.get_physical_groups()) {
            MatrixXd V;
            MatrixXi F;
            msh.get_VF(group, V, F);
            tag_names.push_back(group.name);
            if ((has_tets && group.dim == 2) || (group.dim == 1)) { // envelope surface
                if (V_env.size() != 0) {
                    log_and_throw_error("Multiple envelope groups found in {}", path);
                }
                input_data.V_envelope = V;
                input_data.F_envelope = F;
            } else {
                Vs.push_back(V);
                Fs.push_back(F);
            }
        }

        // combine vertices
        if (Vs.empty()) {
            log_and_throw_error("No vertices found in {}", path);
        }
        auto& V = input_data.V_input;
        for (const auto& VV : Vs) {
            if (VV.rows() == 0) {
                continue;
            }
            size_t n = V.rows();
            V.conservativeResize(n + VV.rows(), VV.cols());
            V.block(n, 0, VV.rows(), V.cols()) = VV;
        }

        // combine elements
        if (has_tets) { // 3d
            if (Fs.empty()) {
                log_and_throw_error("No tets found in {}", path);
            }

            std::map<simplex::Tet, size_t> ids;
            std::vector<Vector4i> tets;

            for (const auto& TT : Fs) {
                for (size_t i = 0; i < TT.rows(); i++) {
                    const Vector4i& t = TT.row(i);
                    simplex::Tet tet_simp(t(0), t(1), t(2), t(3));
                    if (ids.count(tet_simp) == 0) {
                        ids[tet_simp] = tets.size();
                        tets.push_back(t);
                    }
                }
            }

            auto& T = input_data.T_input;
            T.resize(tets.size(), 4);
            for (size_t i = 0; i < tets.size(); i++) {
                T.row(i) = tets[i];
            }

            input_data.T_input_tags.resize(input_data.T_input.rows(), Fs.size());
            for (size_t i = 0; i < Fs.size(); i++) {
                for (size_t j = 0; j < Fs[i].rows(); j++) {
                    const Vector4i& t = Fs[i].row(j);
                    simplex::Tet tet_simp(t(0), t(1), t(2), t(3));
                    size_t t_id = ids[tet_simp];
                    input_data.T_input_tags.coeffRef(t_id, i) = 1;
                }
                input_data.tag_names.push_back(tag_names[i]);
            }
        } else { // 2d
            // resize vert matrices
            auto& Vi = input_data.V_input;
            Vi = Vi.block(0, 0, Vi.rows(), 2).eval();
            auto& Ve = input_data.V_envelope;
            if (Ve.size() > 0) {
                Ve = Ve.block(0, 0, Ve.rows(), 2).eval();
            }

            if (Fs.empty()) {
                log_and_throw_error("No faces found in {}", path);
            }

            std::map<simplex::Face, size_t> ids;
            std::vector<Vector3i> faces;

            for (const auto& FF : Fs) {
                for (size_t i = 0; i < FF.rows(); i++) {
                    const Vector3i& f = FF.row(i);
                    const simplex::Face face_simp(f(0), f(1), f(2));
                    if (ids.count(face_simp) == 0) {
                        ids[face_simp] = faces.size();
                        faces.push_back(f);
                    }
                }
            }

            auto& T = input_data.T_input;
            T.resize(faces.size(), 3);
            for (size_t i = 0; i < faces.size(); i++) {
                T.row(i) = faces[i];
            }

            input_data.T_input_tags.resize(input_data.T_input.rows(), Fs.size());
            for (size_t i = 0; i < Fs.size(); i++) {
                for (size_t j = 0; j < Fs[i].rows(); j++) {
                    const Vector3i& f = Fs[i].row(j);
                    const simplex::Face face_simp(f(0), f(1), f(2));
                    size_t f_id = ids[face_simp];
                    input_data.T_input_tags.coeffRef(f_id, i) = 1;
                }
                input_data.tag_names.push_back(tag_names[i]);
            }
        }
        return input_data;
    }

    if (ph_vol && ph_env) {
        logger().info("Found ImageVolume and EnvelopeSurface");

        // check dimensions
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
            logger().info("Read 2D input");
            msh.get_VF(2, ph_vol.value().tag, input_data.V_input, input_data.T_input);
            msh.get_VF(1, ph_env.value().tag, input_data.V_envelope, input_data.F_envelope);
            auto& Vi = input_data.V_input;
            Vi = Vi.block(0, 0, Vi.rows(), 2).eval();
            auto& Ve = input_data.V_envelope;
            Ve = Ve.block(0, 0, Ve.rows(), 2).eval();

        } else {
            logger().info("Read 3D input");
            msh.get_VF(2, ph_vol.value().tag, input_data.V_input, input_data.T_input);
            msh.get_VF(2, ph_env.value().tag, input_data.V_envelope, input_data.F_envelope);
        }
    } else { // read direct from elements
        logger().info(
            "Could not find pysical groups ImageVolume and EnvelopeSurface. Reading only tet data "
            "(in 2D face data) from MSH.");

        if (has_faces && !has_tets) {
            logger().info("Read 2D input");
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
            logger().info("Read 3D input");
            input_data.V_input.resize(msh.get_num_tet_vertices(), 3);
            input_data.T_input.resize(msh.get_num_tets(), 4);
            msh.extract_tet_vertices([&input_data](size_t i, double x, double y, double z) {
                input_data.V_input.row(i) << x, y, z;
            });
            msh.extract_tets([&input_data](size_t i, size_t v0, size_t v1, size_t v2, size_t v3) {
                input_data.T_input.row(i) << (int)v0, (int)v1, (int)v2, (int)v3;
            });
            positive_orientation_3D(input_data.V_input, input_data.T_input);
        }
    }

    logger().warn(
        "No physical groups found. If multiple layers are present they will be flattened");

    // read data arrays
    if (input_data.T_input.cols() == 4) {
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
            msh.extract_tet_attribute(
                attr_name,
                [&input_tags, &tag_id](size_t i, std::vector<double> val) {
                    assert(val.size() == 1);
                    input_tags(i, tag_id) = val[0];
                });
        }

        // convert tags to "binary"
        MatrixXi input_tags_bin;
        for (int j = 0; j < input_tags.cols(); j++) {
            VectorXi img = input_tags.col(j);
            int max_id = img.maxCoeff();
            MatrixXi img_bin;
            img_bin.resize(img.rows(), max_id);
            img_bin.setZero();
            for (int i = 0; i < img.size(); i++) {
                if (img(i) == 0) {
                    continue;
                }
                img_bin(i, img[i] - 1) = 1;
            }
            int nc = input_tags_bin.cols();
            input_tags_bin.conservativeResize(
                img_bin.rows(),
                input_tags_bin.cols() + img_bin.cols());
            input_tags_bin.block(0, nc, img_bin.rows(), img_bin.cols()) = img_bin;
        }
        input_data.T_input_tags = input_tags_bin.sparseView();
        for (int j = 0; j < input_tags_bin.cols(); j++) {
            input_data.tag_names.push_back("tag_" + std::to_string(j));
        }
    } else {
        assert(input_data.T_input.cols() == 3);
        int faces_tags_count = 0;
        for (const std::string& attr_name : msh.get_all_element_attribute_names()) {
            if (attr_name.substr(0, 4) == "tag_") {
                faces_tags_count++;
            }
        }

        MatrixXi input_tags;
        input_tags.resize(input_data.T_input.rows(), faces_tags_count);
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
        for (int j = 0; j < input_tags.cols(); j++) {
            VectorXi img = input_tags.col(j);
            int max_id = img.maxCoeff();
            MatrixXi img_bin;
            img_bin.resize(img.rows(), max_id);
            img_bin.setZero();
            for (int i = 0; i < img.size(); i++) {
                if (img(i) == 0) {
                    continue;
                }
                img_bin(i, img[i] - 1) = 1;
            }
            int nc = input_tags_bin.cols();
            input_tags_bin.conservativeResize(
                img_bin.rows(),
                input_tags_bin.cols() + img_bin.cols());
            input_tags_bin.block(0, nc, img_bin.rows(), img_bin.cols()) = img_bin;
        }
        input_data.T_input_tags = input_tags_bin.sparseView();
        for (int j = 0; j < input_tags_bin.cols(); j++) {
            input_data.tag_names.push_back("tag_" + std::to_string(j));
        }
    }

    return input_data;
}


} // namespace wmtk::components::topological_offset
