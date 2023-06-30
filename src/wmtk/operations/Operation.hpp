#pragma once
#include <wmtk/Tuple.hpp>
#include <type_traits>

namespace wmtk {
class Operation
{
public:

    // main entry point of the operator by the scheduler
    bool operator()();
    virtual std::string name() const = 0;


    Operation();
    virtual ~Operation();

    virtual std::vector<double> priority() const { return {0}; }
    virtual std::vector<Tuple> modified_triangles() const = 0;

protected:
    virtual bool execute() = 0;
    virtual bool before() const = 0;
    virtual bool after() const = 0;

    virtual void assign(const Tuple& t) {}
    virtual void mark_failed() {}

    AccessorScope m_scope;

};



} // namespace wmtk

