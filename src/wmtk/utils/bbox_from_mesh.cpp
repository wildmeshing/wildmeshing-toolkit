#include "bbox_from_mesh.hpp"

#include <variant>

#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::utils {


Eigen::MatrixXd bbox_from_mesh(const attribute::MeshAttributeHandle& position_handle)
{
    const Mesh& m = position_handle.mesh();

    const int64_t dim = position_handle.dimension();

    Eigen::MatrixXd bbox;
    bbox.resize(2, dim);
    for (int64_t d = 0; d < dim; ++d) {
        bbox(0, d) = std::numeric_limits<double>::max();
        bbox(1, d) = std::numeric_limits<double>::lowest();
    }

    std::visit(
        [&bbox, &dim, &m](const auto& v) -> void {
            using T = typename std::decay_t<decltype(v)>::Type;
            const auto p_acc = m.create_const_accessor<T>(v);
            for (const Tuple& t : m.get_all(PrimitiveType::Vertex)) {
                const auto p = p_acc.const_vector_attribute(t);
                for (int64_t d = 0; d < dim; ++d) {
                    if constexpr (std::is_same_v<T, double>) {
                        bbox(0, d) = std::min(bbox(0, d), p[d]);
                        bbox(1, d) = std::max(bbox(1, d), p[d]);
                    } else if constexpr (std::is_same_v<T, Rational>) {
                        bbox(0, d) = std::min(bbox(0, d), p[d].to_double());
                        bbox(1, d) = std::max(bbox(1, d), p[d].to_double());
                    } else {
                        log_and_throw_error("Cannot compute bbox. Unknown position handle type.");
                    }
                }
            }
        },
        position_handle.handle());

    // std::cout << "BBOX:\n" << bbox << std::endl;

    return bbox;
}

double bbox_diagonal_from_mesh(const attribute::MeshAttributeHandle& position_handle)
{
    const Eigen::MatrixXd bbox = bbox_from_mesh(position_handle);

    const double d = (bbox.row(0) - bbox.row(1)).norm();

    return d;
}

} // namespace wmtk::utils