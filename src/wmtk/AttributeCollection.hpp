#pragma once

#include <wmtk/utils/VectorUtils.h>
#include <wmtk/utils/Logger.hpp>

#include <tbb/concurrent_vector.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <map>
#include <optional>
#include <vector>

namespace wmtk {
class AbstractAttributeContainer
{
public:
    virtual ~AbstractAttributeContainer() = default;
    virtual void move(size_t from, size_t to){};
    virtual void resize(size_t){};
    virtual void resize(){};
    virtual void rollback(){};
};


template <typename T>
struct AttributeCollection : public AbstractAttributeContainer
{
    void move(size_t from, size_t to) override { m_attributes[to] = std::move(m_attributes[from]); }
    void resize(size_t s) override
    {
        m_attributes.grow_to_at_least(s);
        // TODO: in Concurrent, vertex partition id, vertex mutex should be part of attribute
    }

    bool assign(size_t to, T&& val)
    {
        m_attributes[to] = val;
        m_rollback_list[to] = val;
        // TODO: are locks necessary? not now.
        return true;
    }

    void rollback() override
    {
        for (auto& [i, v] : m_rollback_list) {
            m_attributes[i] = std::move(v);
        }
        m_rollback_list.clear();
    }
    
    size_t size() const {
        return m_attributes.size();
    }
    std::map<size_t, T> m_rollback_list;
    // experimenting with tbb, could be templated as well.
    tbb::concurrent_vector<T> m_attributes;
};
} // namespace wmtk