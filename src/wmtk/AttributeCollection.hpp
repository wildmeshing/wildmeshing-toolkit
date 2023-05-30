#pragma once

#include <wmtk/utils/VectorUtils.h>
#include <wmtk/utils/Logger.hpp>

#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>

#include <wmtk/utils/AttributeCollectionProtectRAII.h>
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
class AttributeCollectionReplayer;
template <typename T>
class AttributeCollectionSerialization;


class AbstractAttributeCollection
{
public:
    AbstractAttributeCollection();
    virtual ~AbstractAttributeCollection();
    virtual void move(size_t /*from*/, size_t to) = 0;
    // resize an attribute, including shrinking its size
    // in potentially parallel code use grow_to_at_least insetad
    virtual void resize(size_t) = 0;
    // parallel-safe resize of teh data that can only increase the size of the held data
    virtual void grow_to_at_least(size_t) = 0;
    virtual size_t size() const = 0;
    virtual void rollback() = 0;
    virtual void begin_protect();
    virtual std::optional<size_t> end_protect();


    bool is_in_protect() const;


    template <typename T>
    friend class AttributeCollectionSerialization;
    friend class AttributeCollectionRecorder;
    friend class AttributeCollectionReplayer;

    // checks
    bool has_recorders() const { return !recorder_ptrs.empty(); }
    void add_recorder(AttributeCollectionRecorder*);
    void remove_recorder(AttributeCollectionRecorder*);

protected:
    mutable tbb::enumerable_thread_specific<std::optional<size_t>> m_rollback_size;

private:
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
        // disallow unprotected access with an active recorder
        if (!is_in_protect()) {
            assert(!has_recorders());
        }
        m_attributes[to] = std::move(m_attributes[from]);
    }
    void resize(size_t s) override
    {
        // disallow unprotected access with an active recorder
        if (!is_in_protect()) {
            assert(!has_recorders());

        } else {
            m_rollback_size.local() = m_attributes.size();
            if (has_recorders()) {
                auto& rollback = m_rollback_list.local();
                // if we are shrinking then indices lie like
                // [0, ... ,s, ... ,m_attributes.size()-1]
                // we need to copy out these elements:
                // [        s, ... ,m_attributes.size()-1]
                for (size_t j = s; j < m_attributes.size(); ++j) {
                    rollback[j] = m_attributes[j];
                }
            }
        }
        m_attributes.resize(s);
        // if (m_attributes.size() > s) {
        //     m_attributes.resize(s);
        //     m_attributes.shrink_to_fit();
        // }
        // TODO: in Concurrent, vertex partition id, vertex mutex should be part of attribute
    }
    auto begin() { return m_attributes.begin(); }
    auto end() { return m_attributes.end(); }
    auto begin() const { return m_attributes.begin(); }
    auto end() const { return m_attributes.end(); }

    void shrink_to_fit() { m_attributes.shrink_to_fit(); }

    void grow_to_at_least(size_t s) override
    {
        // disallow unprotected access with an active recorder
        if (!is_in_protect()) {
            assert(!has_recorders());
        }
        m_attributes.grow_to_at_least(s);
    }

    /**
     * @brief retrieve the protected attribute data on operation-fail
     *
     */
    void rollback() override
    {
        std::optional<size_t>& rollback_size_opt = m_rollback_size.local();
        assert(rollback_size_opt.has_value());
        grow_to_at_least(rollback_size_opt.value());
        for (auto& [i, v] : m_rollback_list.local()) {
            m_attributes[i] = std::move(v);
        }
        // reset the recording state, which is implemented by rollback_size having a value
        m_rollback_size.local().reset();
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
        if (is_in_protect()) {
            m_rollback_list.local().emplace(i, m_attributes[i]);
        } else {
            // disallow unprotected access with an active recorder
            assert(!has_recorders());
        }
        return m_attributes[i];
    }

    const T& at(size_t i) const { return m_attributes[i]; }

    size_t size() const override { return m_attributes.size(); }
    tbb::enumerable_thread_specific<std::map<size_t, T>> m_rollback_list;
    // experimenting with tbb, could be templated as well.
    //private:
    mutable tbb::concurrent_vector<T> m_attributes;
};
} // namespace wmtk
