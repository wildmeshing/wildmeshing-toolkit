# Attributes


#### Accessors
Accessors are per-thread mechanisms for interacting with individual attributes.
They are constructed by a `create_accessor` or `create_const_accessor` request from a mesh using a [handle](#Handles).
A single instance of this class should NEVER persist outside of a single
thread. Handles can survive between threads and in single-threaded functinos we
construct an accessor from the handle to provide access to attributes.

#### Handles
Handles are thread-safe representations of attributes. There are a few types of handles:
* `SmartAttributeHandle<T>`: fully encodes an attribute, including which mesh
  it comes from. Can construct accessors from it directly.
* `MeshAttributeHandle<T>`: encodes an attribute in some mesh, useful when the
  mesh in use is unambiguous.
* `AttributeHandle`: an internal handle that does not understand primitive type
  or scalar type of an attribute.

