#include "read_image_msh.hpp"

#include <wmtk/utils/io.hpp>

namespace wmtk::components::tet_implicits {

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
                      "tet data from MSH.");

        V_input.resize(msh.get_num_tet_vertices(), 3);
        T_input.resize(msh.get_num_tets(), 4);
        msh.extract_tet_vertices(
            [&V_input](size_t i, double x, double y, double z) { V_input.row(i) << x, y, z; });
        msh.extract_tets([&T_input](size_t i, size_t v0, size_t v1, size_t v2, size_t v3) {
            T_input.row(i) << (int)v0, (int)v1, (int)v2, (int)v3;
        });

        // check for inversion
        {
            const Vector3d p0 = V_input.row(T_input(0, 0));
            const Vector3d p1 = V_input.row(T_input(0, 1));
            const Vector3d p2 = V_input.row(T_input(0, 2));
            const Vector3d p3 = V_input.row(T_input(0, 3));

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
                T_input.col(2).swap(T_input.col(3));
            }
        }
    }

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
}

} // namespace wmtk::components::tet_implicits