# The index convenction is arbitrary, 
# orientation is not important here

vertices = [{0},{1},{2}]

edges = [
        {1,2},
        {2,0},
        {0,1},
]

edges_oriented = [
        [1,2],
        [2,0],
        [0,1],    
]

s = [vertices,edges]
simplices = s

# Generates all possible tuples, note that some might not be valid
def all_tuples(s):
    return [[i,j] for i in range(len(s[0])) for j in range(len(s[1]))]

# Check if a tuple is valid, i.e. if the simplex of dimension d-1 is a face of a simplex of dimension d
def valid_tuple(t,s):
    return s[0][t[0]].issubset(s[1][t[1]]) 

# Check if a tuple is ccw
def ccw_tuple(t,s):
    assert(valid_tuple(t,s))
    assert (edges_oriented[t[1]][0] in s[0][t[0]]) or (edges_oriented[t[1]][1] in s[0][t[0]])
    return edges_oriented[t[1]][0] in s[0][t[0]]

# valid = [t for t in all_tuples(s) if valid_tuple(t,s)]

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
            sv[i] = [-1,-1]
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

# table_ccw("auto_2d_table_ccw",s)

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

#table_complete_tuple("complete_vertex",0,s)

def switch_index(i,d,s,table):
    pass
    