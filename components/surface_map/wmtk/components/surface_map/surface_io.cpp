#include "surface_io.hpp"

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/io.hpp>

namespace wmtk::components::surface_map {

SurfaceData read_surface_msh(const std::string& path)
{
    MshData msh;
    try {
        msh.load(path);
    } catch (const std::exception& e) {
        log_and_throw_error("Could not read {}: {}", path, e.what());
    }
    SurfaceData s;
    s.vertices.resize(msh.get_num_face_vertices());
    s.faces.resize(msh.get_num_faces());
    msh.extract_face_vertices(
        [&s](size_t i, double x, double y, double z) { s.vertices[i] = Eigen::Vector3d(x, y, z); });
    msh.extract_faces([&s](size_t i, size_t a, size_t b, size_t c) { s.faces[i] = {{a, b, c}}; });
    if (s.faces.empty()) log_and_throw_error("{} holds no triangles", path);
    for (const std::string& name : msh.get_face_attribute_names()) {
        auto& field = s.face_fields[name];
        field.resize(s.faces.size());
        msh.extract_face_attribute(name, [&field](size_t i, const std::vector<double>& data) {
            field[i] = data;
        });
    }
    return s;
}

void write_surface_msh(const std::string& path, const SurfaceData& s)
{
    MshData msh;
    msh.add_face_vertices(s.vertices.size(), [&s](size_t i) { return s.vertices[i]; });
    msh.add_faces(s.faces.size(), [&s](size_t i) { return s.faces[i]; });
    for (const auto& [name, field] : s.face_fields) {
        const size_t n = field.empty() ? 0 : field.front().size();
        if (n == 1) {
            msh.add_face_attribute<1>(name, [&field](size_t i) { return field[i][0]; });
        } else if (n == 3) {
            msh.add_face_attribute<3>(name, [&field](size_t i) {
                return Eigen::Vector3d(field[i][0], field[i][1], field[i][2]);
            });
        } else if (n == 9) {
            msh.add_face_attribute<9>(name, [&field](size_t i) {
                Eigen::Matrix<double, 9, 1> v;
                for (int k = 0; k < 9; ++k) v[k] = field[i][k];
                return v;
            });
        } else {
            logger().warn("{}: field {} has {} components per face; not written", path, name, n);
        }
    }
    msh.save(path);
}

} // namespace wmtk::components::surface_map
