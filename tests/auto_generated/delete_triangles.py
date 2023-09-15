import igl
import numpy as np
from collections import defaultdict

def is_mesh_edge_manifold(faces):
    from collections import defaultdict
    edge_count = defaultdict(int)
    for face in faces:
        for i in range(3):
            edge = tuple(sorted([face[i], face[(i+1)%3]]))
            edge_count[edge] += 1
    return all(count <= 2 for count in edge_count.values())

def check_vertex_manifold(vertex, faces, vertex_to_faces):
    connected_faces = vertex_to_faces[vertex]
    visited_faces = set()
    if (len(connected_faces) == 0):
        return True
    stack = [next(iter(connected_faces))]

    while stack:
        current_face = stack.pop()
        visited_faces.add(current_face)

        for idx in faces[current_face]:
            if idx != vertex:
                neighbor_faces = vertex_to_faces[idx] & vertex_to_faces[vertex] - visited_faces
                stack.extend(neighbor_faces)

    return visited_faces == connected_faces

def is_mesh_vertex_manifold(faces, vertex_to_check = None):
    from collections import defaultdict

    vertex_to_faces = defaultdict(set)
    for i, face in enumerate(faces):
        for vertex in face:
            vertex_to_faces[vertex].add(i)
    
    if vertex_to_check is None:
        vertex_to_check = vertex_to_faces.keys()
        
    for vertex in vertex_to_check:
        if (not check_vertex_manifold(vertex, faces, vertex_to_faces)):
            return False

    return True

def is_mesh_manifold(faces):
    return is_mesh_edge_manifold(faces) and is_mesh_vertex_manifold(faces)

def remove_redundant_vertices(vertices, faces):
    unique_vertices, indices = np.unique(faces, return_inverse=True)
    new_vertices = vertices[unique_vertices]
    new_faces = indices.reshape(faces.shape)
    return new_vertices, new_faces

# a better version to delete one by one
def delete_one_by_one_better(faces, prob=0.1, max_fail_cnt = 500):
    target_size = faces.shape[0] * (1 - prob)
    # random pick one to be false
    mask = np.ones(faces.shape[0], dtype=bool)
    vertex_to_faces = defaultdict(set)
    for i, face in enumerate(faces):
        for vertex in face:
            vertex_to_faces[vertex].add(i)
    current_size = faces.shape[0]
    fail_cnt = 0
    while current_size > target_size:
        index_to_delete = np.random.randint(faces.shape[0])
       
        while not mask[index_to_delete]:
            index_to_delete = np.random.randint(faces.shape[0])
        mask[index_to_delete] = False
        for vid in faces[index_to_delete]:
            vertex_to_faces[vid].remove(index_to_delete)
        
        flag = True
        # check if still vertex manifold
        for vid in faces[index_to_delete]:
            flag &= check_vertex_manifold(vid, faces, vertex_to_faces)
        if not flag:
            mask[index_to_delete] = True
            for vid in faces[index_to_delete]:
                vertex_to_faces[vid].add(index_to_delete)
            fail_cnt += 1
        else:
            current_size -= 1
            fail_cnt = 0
        if fail_cnt > max_fail_cnt:
            print("consecutive fail to delete one more than {} times".format(max_fail_cnt))
            print("current size: {}, current size / input size: {}".format(current_size, current_size / faces.shape[0]))
            break
    return faces[mask]

WMTK_DATA_DIR = "../../data/"

model_name = "blub"
v,_,_,f,_,_ = igl.read_obj(WMTK_DATA_DIR + model_name + ".obj")

del_prob = 0.5
v_new, f_new = remove_redundant_vertices(v, delete_one_by_one_better(f, del_prob, 500))

if (is_mesh_manifold(f_new)):
    igl.write_obj("{}_{}.obj".format(model_name, 1-del_prob), v_new, f_new)