# The index convenction is arbitrary, 
# orientation is not important here

vertices = [{0},{1},{2},{3}]

edges = [
        {0,1},
        {0,2},
        {0,3},
        {1,2},
        {1,3},
        {2,3}
]

faces = [
        {1,2,3},
        {0,3,2},
        {0,1,3},
        {0,2,1}
    ]

faces_oriented = [
        [1,2,3],
        [0,3,2],
        [0,1,3],
        [0,2,1]
    ]


s = [vertices,edges,faces]


simplices = s

# Generates all possible tuples, note that some might not be valid
def all_tuples(s):
    return [[i,j,k] for i in range(len(s[0])) for j in range(len(s[1])) for k in range(len(s[2]))]

# Check if a tuple is valid, i.e. if the simplex of dimension d-1 is a face of a simplex of dimension d
def valid_tuple(t,s):
    return s[0][t[0]].issubset(s[1][t[1]]) and s[1][t[1]].issubset(s[2][t[2]]) 

# Check if a tuple is ccw
def ccw_tuple(t,s):
    assert(valid_tuple(t,s))
    #B = [0,1,2,3,0,1,2,3,0,1,2,3,0,1,2,3]
    oriented_face = faces_oriented[t[2]]*3
    edge = s[1][t[1]]
    vertex = s[0][t[0]]
    e1 = edge - vertex
    B = oriented_face
    A = [list(vertex)[0], list(e1)[0]]
    return any(A == B[i:i + len(A)] for i in range(len(B)-len(A) + 1)) # check for sublist

# len([t for t in all_tuples(s) if (valid_tuple(t,s) and ccw_tuple(t,s))])

# Enumerates all valid tuples similar to t, but with a different value in the slot d
# There must be 2 of them, and this function returns the one that is different than t
def switch(t,d,s):
    candidates = []
    for i in range(len(s[d])):
        t2 = t.copy()
        t2[d] = i
        candidates.append(t2)
    valid = [x for x in candidates if valid_tuple(x,s)]
    assert(len(valid) == 2)
    valid.remove(t)
    assert(len(valid) == 1)
    return valid[0]

# Builds a table for the switch operation of dimension d
def table(d,s):
    # Generates a table for switch_vertex
    sv = all_tuples(s)

    for i in range(len(sv)):
        if not valid_tuple(sv[i],s):
            sv[i] = [-1,-1,-1]
        else:
            sv[i] = switch(sv[i],d,s)

    return sv

# Builds a table for the switch operation of dimension d
def table_ccw(s):
    # Generates a table for switch_vertex
    sv = all_tuples(s)
    out = [-1 for i in sv]

    for i in range(len(sv)):
        if not valid_tuple(sv[i],s):
            out[i] = -1
        else:
            if ccw_tuple(sv[i],s):
                out[i] = 1
            else:
                out[i] = 0
            
    return out

# table_ccw("auto_3d_table_ccw",s)

def table_complete_tuple(d,s):
    # Generates a table for switch_vertex
    sv = [t for t in all_tuples(s) if (valid_tuple(t,s) and ccw_tuple(t,s))]
    out = []
    for c in range(len(s[d])):
        for i in range(len(sv)):
            if c == sv[i][d]:
                out.append(sv[i])
                break
    
    assert(len(out) == len(s[d]))
            
    return out
