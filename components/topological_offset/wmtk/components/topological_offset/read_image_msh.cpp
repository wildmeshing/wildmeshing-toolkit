#include "read_image_msh.hpp"
#include <wmtk/utils/io.hpp>


namespace wmtk::components::topological_offset {


void read_image_msh(
    const std::string& path,
    MatrixXd& V_input,
    MatrixXi& F_input,
    MatrixXd& F_input_tags)
{
    MshData msh;
    msh.load(path);

    if (msh.get_num_tets() == 0) { // 2d triangle mesh
        V_input.resize(msh.get_num_face_vertices(), 2);
        F_input.resize(msh.get_num_faces(), 3);
        msh.extract_face_vertices(
            [&V_input](size_t i, double x, double y, double z) { V_input.row(i) << x, y; });
        msh.extract_faces([&F_input](size_t i, size_t v0, size_t v1, size_t v2) {
            F_input.row(i) << v0, v1, v2;
        });

        { // inversion check
            const Vector2d p0 = V_input.row(F_input(0, 0));
            const Vector2d p1 = V_input.row(F_input(0, 1));
            const Vector2d p2 = V_input.row(F_input(0, 2));

            igl::predicates::exactinit();
            auto res = igl::predicates::orient2d(p0, p1, p2);
            int result;
            if (res == igl::predicates::Orientation::POSITIVE) {
                result = 1;
            } else if (res == igl::predicates::Orientation::NEGATIVE) {
                result = -1;
            } else {
                log_and_throw_error(
                    "First tet is degenerate! Vertices: \n{},\n{},\n{}",
                    p0,
                    p1,
                    p2);
            }

            if (result <= 0) {
                logger().warn("First triangle of input is inverted -> invert all triangles.");
                F_input.col(1).swap(F_input.col(2));
            }
        }
    } else { // 3d tet mesh
        V_input.resize(msh.get_num_tet_vertices(), 3);
        F_input.resize(msh.get_num_tets(), 4);
        msh.extract_tet_vertices(
            [&V_input](size_t i, double x, double y, double z) { V_input.row(i) << x, y, z; });
        msh.extract_tets([&F_input](size_t i, size_t v0, size_t v1, size_t v2, size_t v3) {
            F_input.row(i) << v0, v1, v2, v3;
        });

        { // inversion check
            const Vector3d p0 = V_input.row(F_input(0, 0));
            const Vector3d p1 = V_input.row(F_input(0, 1));
            const Vector3d p2 = V_input.row(F_input(0, 2));
            const Vector3d p3 = V_input.row(F_input(0, 3));

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

            if (result >= 0) {
                logger().warn("First tet of input is inverted -> invert all tets.");
                F_input.col(2).swap(F_input.col(3));
            }
        }
    }

    // read tag data
    if (F_input.cols() == 4) { // tet mesh
        int tet_tags_count = 0;
        for (const std::string& attr_name : msh.get_tet_attribute_names()) {
            if (attr_name.substr(0, 4) == "tag_") {
                tet_tags_count++;
            }
        }

        F_input_tags.resize(F_input.rows(), tet_tags_count);
        for (const std::string& attr_name : msh.get_tet_attribute_names()) {
            if (attr_name.substr(0, 4) != "tag_") {
                continue;
            }
            const int tag_id = std::stoi(attr_name.substr(4));
            msh.extract_tet_attribute(
                attr_name,
                [&F_input_tags, &tag_id](size_t i, std::vector<double> val) {
                    F_input_tags(i, tag_id) = val[0];
                });
        }
    } else {
        int face_tags_count = 0;
        for (const std::string& attr_name : msh.get_all_element_attribute_names()) {
            if (attr_name.substr(0, 4) == "tag_") {
                face_tags_count++;
            }
        }

        F_input_tags.resize(F_input.rows(), face_tags_count);
        for (const std::string& attr_name : msh.get_all_element_attribute_names()) {
            if (attr_name.substr(0, 4) != "tag_") {
                continue;
            }
            const int tag_id = std::stoi(attr_name.substr(4));
            msh.extract_face_attribute(
                attr_name,
                [&F_input_tags, &tag_id](size_t i, std::vector<double> val) {
                    F_input_tags(i, tag_id) = val[0];
                });
        }
    }
}


} // namespace wmtk::components::topological_offset