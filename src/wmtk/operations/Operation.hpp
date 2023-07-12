#pragma once
#include <type_traits>
#include <wmtk/Tuple.hpp>

namespace wmtk {
class Operation
{
public:
    Operation(Mesh& mesh);
    // main entry point of the operator by the scheduler
    bool operator()();
    virtual ~Operation();

    virtual std::vector<double> priority() const;
    virtual std::vector<Tuple> modified_triangles() const = 0;

protected:
    virtual bool execute() = 0;
    virtual bool before() const = 0;
    virtual bool after() const = 0;


    Mesh& m_mesh;
};


} // namespace wmtk

