# Attribute Classes

We describe the structure of attributes and their usage. All access to attributes is guarded by several layers, where each one fulfills a different purpose. From top to bottom the layers are:

1. The `Mesh` contains an `AttributeManager`.
2. The `AttributeManager` contains type specific `TypedAttributeManager`.
3. Every `TypedAttributeManager` stores a vector of `CachingAttribute`s and a map from their names to their handles.
4. The `CachingAttributes` provide utility to revert changes on `Attribute`s.
5. The `Attribute` class is a wrapper around the raw attribute data that provides read and write functionality.
6. For referencing and accessing attributes, we use handles and accessors.

In the following, we provide a summary of the most important concepts that were used in all attribute-related classes.

#### Attributes

All properties of a mesh (including its topology) are stored in attributes. At its core, attributes are vectors:

```C++
template <typename T>
class Attribute {
    std::vector<T> m_data;
    int64_t m_dimension;
    std::string m_name;

    Eigen::Matrix<T>::MapType vector_attribute(int64_t index);
    T& scalar_attribute(int64_t index);
};
```

Attributes have a `dimension`. If the `dimension = 1` then the attribute is a scalar attribute, otherwise, it is a vector attribute. This distinction is necessary for accessing the attribute. While the former case returns just a scalar, the other one returns an `Eigen::Map` (similar to a `Eigen::Vector` but without creating a copy).

Changes on Attributes can be reverted through the `CachingAttribute` that inherits from `Attribute`. The `CachingAttribute` holds scopes that can be pushed and popped:

```C++
template <typename T>
class CachingAttribute : public Attribute<T>
{
    void push_scope();
    void pop_scope(bool preserve_changes);
}
```

The scopes work as a recording mechanism. When a new scope is pushed with `push_scope()` all changes to the attribute are tracked. When the scope is popped, the changes can either be accepted with `pop_scope(true)` or discarded with `pop_scope(false)`.

#### Managers

All attributes of the same type are stored in a `TypedAttributeManager`. The manager deals with the registration and removal of attributes. Note that while attributes can be removed, the vector of attributes is not resized. Otherwise, `AttributeHandle`s would point to the wrong data. If the vector should be resized, e.g., for serialization/writing to file, `clear_dead_attributes()` needs to be called.

```C++
template <typename T>
class TypedAttributeManager
{
    AttributeHandle register_attribute(/*...*/);
    void remove_attribute(const AttributeHandle& attribute);

    std::vector<std::unique_ptr<CachingAttribute<T>>> m_attributes;
}
```

Note that the `TypedAttributeManager` is just a data storage object and does not know anything about meshes or primitive types, e.g., vertex or edge.

The `AttributeManager` is a container for all `TypedAttributeManager`.

```C++
class AttributeManager
{
    std::vector<TypedAttributeManager<char>> m_char_attributes;
    std::vector<TypedAttributeManager<int64_t>> m_long_attributes;
    std::vector<TypedAttributeManager<double>> m_double_attributes;
    std::vector<TypedAttributeManager<Rational>> m_rational_attributes;
}
```

The `Mesh` contains an `AttributeManager`.

#### Handles

The `TypedAttributeManager` holds a map from names to `AttributeHandle`s. They are only for internal usage within the `TypedAttributeManager` to reference and access attributes.

```C++
class AttributeHandle
{
    int64_t m_index = -1;
}
```

The `TypedAttributeHandle` is the most basic handle used outside the `namespace attribute`. It consists of an `AttributeHandle` and a `PrimitiveType`. Together with the attribute's `Type`, an attribute is uniquely defined for a single `Mesh`.

```C++
template <typename T>
class TypedAttributeHandle
{
    using Type = T;
    AttributeHandle m_base_handle;
    PrimitiveType m_primitive_type;
}
```

The `TypedAttributeHandle` contains a template parameter. To avoid using templates all over the place, there exists the `std::variant` `HandleVariant` in the non-templated `MeshAttributeHandle`. It also contains a reference to its `Mesh`.

```C++
class MeshAttributeHandle
{
    using HandleVariant = std::variant<TypedAttributeHandle<char>,TypedAttributeHandle<int64_t>,
        TypedAttributeHandle<double>,TypedAttributeHandle<wmtk::Rational>>;
    HandleVariant& handle() { return m_handle; }

    Mesh* m_mesh = nullptr;
    HandleVariant m_handle;
}
```

The reference to the `Mesh` makes the handle unique within a multimesh, where multiple meshes might have the "same" attribute, e.g., vertex positions. The `MeshAttributeHandle` is almost always the handle that should be used. The visitor pattern must be used if someone wants to access the `Type` of an attribute from the `MeshAttributeHandle`.

```C++
Mesh m;
MeshAttributeHandle attr;
std::visit(
    [&m](auto&& typed_handle) {
        // get type of the TypedAttributeHandle, e.g., TypedAttributeHandle<double>
        using HandleType = std::decay_t<decltype(typed_handle)>;
        // get attribute data type, e.g., double
        using Type = typename HandleType::Type;

        if constexpr (std::is_same_v<Type, double>) {
            logger().info("Attribute {} is double", m.get_attribute_name(typed_handle));
        }
    },
    attr.handle());
```

More examples for the usage of `std::visit` are given in _test_accessor.cpp_ in the test `"test_attribute_variant"`.

#### Accessors

Handles do not know about the mesh they belong to (except for the `MeshAttributeHandle`). To access an attribute, we need the mesh to get the global ID of a simplex.

```C++
simplex::IdSimplex s;
int64_t idx = mesh.id(s);
m_attribute.scalar_attribute(idx);
```

Also, we want to avoid the indirection of handles. They only store indices, and do not reference the attribute themselves. The `Accessor` class therefore references the attribute directly.

```C++
template<typename T>
class Accessor
{
    TypedAttributeHandle<T> m_handle;
    Mesh& m_mesh;
    CachingAttribute<T>& m_attribute;
}
```

## What Handle / Accessor should I use?

The main advantage of the `MeshAttributeHandle` over all the other handles and the accessor is that it is type independent. If you are writing code where you do not know the data type stored in an attribute, you have to use the `MeshAttributeHandle` in combination with `std::visit`. However, most probably (at least in the first draft), you know exactly what attributes your mesh has and what types they are. In that case, we recommend creating accessors right away and passing those around.

```C++
Mesh m;
MeshAttributeHandle a1 = m.register_attribute<double>("a1", PrimitiveType::Vertex, 1);
Accessor<double> acc_a1 = m.create_accessor<double>(a1);
```
