#include "read_image_msh.hpp"

#include <igl/predicates/predicates.h>
#include <wmtk/utils/io.hpp>

namespace wmtk::components::image_simulation {

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
    const Vector3d p0 = V.row(F(0, 0));
    const Vector3d p1 = V.row(F(0, 1));
    const Vector3d p2 = V.row(F(0, 2));

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

    bool is_inverted = result >= 0;
    if (is_inverted) {
        logger().warn("First face of input is inverted -> invert all faces.");
        F.col(1).swap(F.col(2));
    }
}

void read_image_msh(
    const std::string& path,
    MatrixXd& V_input,
    MatrixXi& T_input,
    MatrixXi& T_input_tag,
    MatrixXd& V_envelope,
    MatrixXi& F_envelope)
{
    MshData msh;
    msh.load(path);

    std::optional<mshio::PhysicalGroup> ph_vol = msh.get_physical_group_by_name("ImageVolume");
    std::optional<mshio::PhysicalGroup> ph_env = msh.get_physical_group_by_name("EnvelopeSurface");

    if (ph_vol && ph_env) {
        logger().info("Found ImageVolume and EnvelopeSurface.");

        if (ph_vol.value().dim != 3) {
            log_and_throw_error(
                "Unexpected dimension {} of pysical group {}",
                ph_vol.value().dim,
                ph_vol.value().name);
        }
        if (ph_env.value().dim != 2) {
            log_and_throw_error(
                "Unexpected dimension {} of pysical group {}",
                ph_env.value().dim,
                ph_env.value().name);
        }

        msh.get_VF(3, ph_vol.value().tag, V_input, T_input);
        msh.get_VF(2, ph_env.value().tag, V_envelope, F_envelope);

    } else {
        // only read volume directly from .msh and ignore other entities
        logger().info("Could not find pysical groups ImageVolume and EnvelopeSurface. Reading only "
                      "tet data (in 2D face data) from MSH.");

        const bool has_tets = msh.get_num_tets() != 0;
        const bool has_faces = msh.get_num_faces() != 0;

        if (has_faces && !has_tets) {
            logger().info("Read 2D input.");
            V_input.resize(msh.get_num_tet_vertices(), 2);
            T_input.resize(msh.get_num_tets(), 3);
            msh.extract_face_vertices(
                [&V_input](size_t i, double x, double y, double z) { V_input.row(i) << x, y;
                if (z != 0) {
                    logger().warn("Ignoring non-zero z component of MSH for 2D input. z = {}", z);
                }
                });
            msh.extract_faces([&T_input](size_t i, size_t v0, size_t v1, size_t v2) {
                T_input.row(i) << (int)v0, (int)v1, (int)v2;
            });

            positive_orientation_2D(V_input, T_input);
        } else if (has_tets) {
            logger().info("Read 3D input.");
            V_input.resize(msh.get_num_tet_vertices(), 3);
            T_input.resize(msh.get_num_tets(), 4);
            msh.extract_tet_vertices(
                [&V_input](size_t i, double x, double y, double z) { V_input.row(i) << x, y, z; });
            msh.extract_tets([&T_input](size_t i, size_t v0, size_t v1, size_t v2, size_t v3) {
                T_input.row(i) << (int)v0, (int)v1, (int)v2, (int)v3;
            });

            positive_orientation_3D(V_input, T_input);
        } else {
            log_and_throw_error("Input has neither 2D nor 3D input");
        }
    }

    if (T_input.cols() == 4) {
        int tets_tags_count = 0;
        for (const std::string& attr_name : msh.get_tet_attribute_names()) {
            if (attr_name.substr(0, 4) == "tag_") {
                ++tets_tags_count;
            }
        }

        T_input_tag.resize(T_input.rows(), tets_tags_count);
        for (const std::string& attr_name : msh.get_tet_attribute_names()) {
            if (attr_name.substr(0, 4) != "tag_") {
                continue;
            }
            const int tag_id = std::stoi(attr_name.substr(4));
            msh.extract_tet_attribute(
                attr_name,
                [&T_input_tag, &tag_id](size_t i, std::vector<double> val) {
                    assert(val.size() == 1);
                    T_input_tag(i, tag_id) = val[0];
                });
        }
    } else {
        assert(T_input.cols() == 3);
        int tets_tags_count = 0;
        for (const std::string& attr_name : msh.get_face_attribute_names()) {
            if (attr_name.substr(0, 4) == "tag_") {
                ++tets_tags_count;
            }
        }

        T_input_tag.resize(T_input.rows(), tets_tags_count);
        for (const std::string& attr_name : msh.get_face_attribute_names()) {
            if (attr_name.substr(0, 4) != "tag_") {
                continue;
            }
            const int tag_id = std::stoi(attr_name.substr(4));
            msh.extract_face_attribute(
                attr_name,
                [&T_input_tag, &tag_id](size_t i, std::vector<double> val) {
                    assert(val.size() == 1);
                    T_input_tag(i, tag_id) = val[0];
                });
        }
    }
}

} // namespace wmtk::components::image_simulation