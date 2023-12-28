#pragma once
#include "Mappable.hpp"

namespace wmtk::multimesh {
class MappableContainer : public Mappable 
{
public:
MappableContainer(const Mesh& m);
    MappableContainer(const MappableContainer&);
    MappableContainer(MappableContainer&&);
    MappableContainer& operator=(const MappableContainer&);
    MappableContainer& operator=(MappableContainer&&);
    ~MappableContainer();


    virtual std::vector<std::shared_ptr<Mappable>> mappables() = 0;

    void add(std::shared_ptr<Mappable> mappable);

    const std::shared_ptr<Mappable>& get(long index) const;
    long size() const;
    bool empty() const;

    std::map<Mesh const*, std::vector<std::shared_ptr<Mappable>>> get_map_mesh_to_invariants();

private:
    std::vector<std::shared_ptr<Mappable>> m_mappables;
};

}
