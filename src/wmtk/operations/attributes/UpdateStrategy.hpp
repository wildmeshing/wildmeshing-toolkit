#pragma once

namespace wmtk {
class Mesh;
class Tuple;
} // namespace wmtk

namespace wmtk::operations::attribute {


class UpdateStrategy
{
public:
    UpdateStrategy();
    ~UpdateStrategy();
    Mesh& mesh();

    virtual void update(const Tuple& a, const Tuple& b) = 0;
    virtual PrimitiveType primitive_type() const = 0;

private:
    Mesh& m_mesh;
};
} // namespace wmtk::operations::attribute
