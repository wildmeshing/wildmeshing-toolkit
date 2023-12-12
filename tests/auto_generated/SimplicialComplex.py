from itertools import chain, combinations
import igl
import numpy as np

class Mesh:
    def __init__(self):
        self.vertices = []    # List of vertices
        self.tetrahedra = []  # List of tetrahedra
        self.faces = set()    # set of triangle faces
        self.edges = set()    # Set of edges
        self.n_vertices = 0

    def add_vertex(self):
        self.vertices.append((self.n_vertices,))
        self.n_vertices += 1

    def add_face(self, f):
        self.faces.add(tuple(sorted(f)))
        self.edges.update([
            tuple(sorted([f[0], f[1]])),
            tuple(sorted([f[1], f[2]])),
            tuple(sorted([f[0], f[2]]))
        ])
    def add_faces(self, faces):
        for f in faces:
            self.add_face(f)

    def add_tetrahedron(self, t):
        self.tetrahedra.append(tuple(sorted(t)))
        self.add_faces([
            (t[0], t[1], t[2]),
            (t[0], t[1], t[3]),
            (t[0], t[2], t[3]),
            (t[1], t[2], t[3])
        ]) 

    def is_face(self, simplex_a, simplex_b):
        return set(simplex_a).issubset(set(simplex_b))
    
    def coface_tetrahedra(self, simplex):
        return [t for t in self.tetrahedra if self.is_face(simplex, t)]

    def coface_faces(self, simplex):
        return [f for f in self.faces if self.is_face(simplex, f)]

    def coface_edges(self, simplex):
        return [e for e in self.edges if self.is_face(simplex, e)]
    
    def open_star(self, simplex):
        simplex = tuple(sorted(simplex))
        ret = []
        if len(simplex) == 1:
            ret.append(simplex) # Vertex
        return ret + self.coface_edges(simplex) + self.coface_faces(simplex) + self.coface_tetrahedra(simplex)

    def simplex_with_boundary(self, s):
        return [tuple(subset) for subset in chain(*[combinations(s, i) for i in range(1, len(s) + 1)])]
    
    def closed_star(self, simplex):
        simplex = tuple(sorted(simplex))
        o_star = self.open_star(simplex)
        c_star = set()
        for s in o_star:
            c_star.update(self.simplex_with_boundary(s))
        return list(c_star)

    def anti_star(self, simplex):
        simplex = tuple(sorted(simplex))
        simplex_bd = set(self.simplex_with_boundary(simplex))
        return [t for t in self.tetrahedra if not simplex_bd & set(self.simplex_with_boundary(t))] + [f for f in self.faces if not simplex_bd & set(self.simplex_with_boundary(f))] + [e for e in self.edges if not simplex_bd & set(self.simplex_with_boundary(e))] + [v for v in self.vertices if not simplex_bd & set(self.simplex_with_boundary(v))]
    
    def link(self, simplex):
        return list(set(self.anti_star(simplex)) & set(self.closed_star(simplex)))

    def top_coface_simplex(self, simplex):
        if len(self.tetrahedra) > 0:
            return self.coface_tetrahedra(simplex)
        else:
            return self.coface_faces(simplex)
    
def load_obj(filename):
    mesh = Mesh()
    v, _, _, f, _, _ = igl.read_obj(filename)
    for i in range(v.shape[0]):
        mesh.add_vertex()
    for i in range(f.shape[0]):
        if (f[i,:].shape[0] == 3):
            mesh.add_face(tuple(f[i,:]))
        elif (f[i,:].shape[0] == 4):
            mesh.add_tetrahedron(tuple(f[i,:]))
    return mesh

def print_sc(sc):
    print("#vertices: ", len([item for item in sc if len(item) == 1]))
    print("#edges: ", len([item for item in sc if len(item) == 2]))
    print("#triangles: ", len([item for item in sc if len(item) == 3]))
    print("#tets: ", len([item for item in sc if len(item) == 4]))
    sorted_lst = sorted(sc, key=lambda x: (len(x),x))
    for item in sorted_lst:
        print(item)
