#pragma once

#include <wmtk/Mesh.hpp>

#include <string>

namespace wmtk {
class MeshUtils
{
private:
    MeshUtils() = default;

public:
    template <typename Mat>
    static MeshAttributeHandle<typename Mat::Scalar> set_matrix_attribute(
        const Mat& data,
        const std::string& name,
        const PrimitiveType& type,
        Mesh& mesh)
    {
        MeshAttributeHandle<typename Mat::Scalar> handle =
            mesh.register_attribute<typename Mat::Scalar>(name, type, data.cols());

        auto accessor = mesh.create_accessor(handle);
        // TODO use tuples
        for (long i = 0; i < data.rows(); ++i) {
            accessor.vector_attribute(i) = data.row(i).transpose();
        }

        return handle;
    }
};
} // namespace wmtk