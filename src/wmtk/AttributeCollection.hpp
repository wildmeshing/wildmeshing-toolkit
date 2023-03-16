#pragma once

#include <wmtk/utils/VectorUtils.h>
#include <wmtk/utils/Logger.hpp>

#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <list>
#include <map>
#include <optional>

namespace wmtk {
/**
 * @brief serving as buffers for attributes data that can be modified by operations
 *
 */
class AttributeCollectionRecorder;
template <typename T>
class AttributeCollectionSerialization;
class AbstractAttributeCollection
{
public:
    AbstractAttributeCollection();
    virtual ~AbstractAttributeCollection();
    virtual void move(size_t /*from*/, size_t to) = 0;
    virtual void resize(size_t) = 0;
    virtual size_t size() const = 0;
    virtual void rollback() = 0;
    virtual void begin_protect();
    virtual std::optional<size_t> end_protect();


    template <typename T>
    friend class AttributeCollectionSerialization;
    friend class AttributeCollectionRecorder;

protected:
    tbb::enumerable_thread_specific<size_t> m_rollback_size;
    tbb::enumerable_thread_specific<bool> in_protected{false};
    // the AttributeCollectionRecorder is in charge of cleaning this up
    // TODO: is there a point in making this concurrent? attribute collections should identify their
    // recorders at times where the number of threads is pretty static?
    std::list<AttributeCollectionRecorder*> recorder_ptrs;
};


template <typename T>
struct AttributeCollection : public AbstractAttributeCollection
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

    void shrink_to_fit() { m_attributes.shrink_to_fit(); }

    void grow_to_at_least(size_t s) { m_attributes.grow_to_at_least(s); }

    bool assign(size_t to, T&& val) // always use this in OP_after
    {
        m_attributes[to] = val;
        if (in_protected.local()) m_rollback_list.local()[to] = val;
        // TODO: are locks necessary? not now.
        return true;
    }
    /**
     * @brief retrieve the protected attribute data on operation-fail
     *
     */
    void rollback() override
    {
        for (auto& [i, v] : m_rollback_list.local()) {
            m_attributes[i] = std::move(v);
        }
        in_protected.local() = false;
        end_protect();
    }
    /**
     * @brief clean local buffers for attribute, and start in_protected
     *
     */
    void begin_protect() override
    {
        m_rollback_list.local().clear();
        AbstractAttributeCollection::begin_protect();
    };
    /**
     * @brief clear local buffers and finish in_protected
     *
     */
    std::optional<size_t> end_protect() override
    {
        std::optional<size_t> ret = AbstractAttributeCollection::end_protect();
        m_rollback_list.local().clear();
        return ret;
    }

    const T& operator[](size_t i) const { return m_attributes[i]; }

    T& operator[](size_t i)
    {
        if (in_protected.local()) {
            m_rollback_list.local().emplace(i, m_attributes[i]);
        }
        return m_attributes[i];
    }

    const T& at(size_t i) const { return m_attributes[i]; }

    size_t size() const override { return m_attributes.size(); }
    tbb::enumerable_thread_specific<std::map<size_t, T>> m_rollback_list;
    // experimenting with tbb, could be templated as well.
    tbb::concurrent_vector<T> m_attributes;
};
} // namespace wmtk
