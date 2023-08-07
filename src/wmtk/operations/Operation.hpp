#pragma once
#include <wmtk/Tuple.hpp>
#include <vector>
#include <type_traits>
#include <string>

namespace wmtk {
    class Mesh;
class Operation
{
public:

    // main entry point of the operator by the scheduler
    bool operator()();
    virtual std::string name() const = 0;


    Operation(Mesh& m);
    virtual ~Operation();

    virtual std::vector<double> priority() const { return {0}; }


    // TODO: spaceship things?
    bool operator<(const Operation& o) const;
    bool operator==(const Operation& o) const;

protected:
    virtual bool execute() = 0;
    // does invariant pre-checks
    virtual bool before() const;
    // does invariant pre-checks
    virtual bool after() const;


    Mesh& m_mesh;

};



} // namespace wmtk

