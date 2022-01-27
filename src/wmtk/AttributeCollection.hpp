#pragma once

#include <wmtk/utils/VectorUtils.h>
#include <wmtk/utils/Logger.hpp>

#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>

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
    virtual void rollback(){};
    virtual void begin_protect(){};
    virtual void end_protect(){};
};


template <typename T>
struct AttributeCollection : public AbstractAttributeContainer
{
    void move(size_t from, size_t to) override
    {
        if (from == to) return;
        m_attributes[to] = std::move(m_attributes[from]);
    }
    void resize(size_t s) override
    {
        m_attributes.grow_to_at_least(s);
        // if (m_attributes.size() > s) {
        //     m_attributes.resize(s);
        //     m_attributes.shrink_to_fit();
        // }
        // TODO: in Concurrent, vertex partition id, vertex mutex should be part of attribute
    }

    bool assign(size_t to, T&& val) // always use this in OP_after
    {
        m_attributes[to] = val;
        if (recording.local()) m_rollback_list.local()[to] = val;
        // TODO: are locks necessary? not now.
        return true;
    }

    void rollback() override
    {
        for (auto& [i, v] : m_rollback_list.local()) {
            m_attributes[i] = std::move(v);
        }
        m_rollback_list.local().clear();
    }

    void begin_protect() override
    {
        m_rollback_list.local().clear();
        recording.local() = true;
    };

    void end_protect() override
    {
        m_rollback_list.local().clear();
        recording.local() = false;
    }

    const T& operator[](size_t i) const { return m_attributes[i]; }

    T& operator[](size_t i)
    {
        if (recording.local()) {
            m_rollback_list.local().emplace(i, m_attributes[i]);
        }
        return m_attributes[i];
    }

    const T& at(size_t i) const { return m_attributes[i]; }

    size_t size() const { return m_attributes.size(); }
    tbb::enumerable_thread_specific<std::map<size_t, T>> m_rollback_list;
    // experimenting with tbb, could be templated as well.
    tbb::concurrent_vector<T> m_attributes;
    tbb::enumerable_thread_specific<bool> recording{false};
};
} // namespace wmtk