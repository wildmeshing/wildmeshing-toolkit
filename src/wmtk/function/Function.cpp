#include "Function.hpp"
using namespace wmtk;
using namespace wmtk::function;

Function::Function(const Mesh& mesh)
    : m_mesh(mesh)
{}

Function::~Function() = default;
