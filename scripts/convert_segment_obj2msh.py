'''
usage:
python convert_segment_obj2msh.py <input> <output>
e.g. python convert_segment_obj2msh.py input.obj output.msh
'''


import numpy as np
import sys

def read_segment_obj(filename):
    v = []
    l = []
    with open(filename, "r") as file:
        for line in file:
            s = line.split()
            if len(s) == 0:
                continue
            if s[0] == 'v':
                assert len(s) == 4
                v.append(np.array([float(s[1]), float(s[2]), float(s[3])]))
            elif s[0] == 'l':
                assert len(s) == 3
                l.append(np.array([int(s[1]) - 1, int(s[2]) - 1]))
            else:
                print("undefined format. line ignored.")
                continue

    return np.array(v), np.array(l)

def write_segment_msh_41(filename, v, l):
    with open(filename, "w") as file:
        file.write("$MeshFormat\n")
        file.write("4.1 0 8\n")
        file.write("$EndMeshFormat\n")
        file.write("$Nodes\n")
        file.write("{} {} {} {}\n".format(1, v.shape[0], 0, v.shape[0] - 1))
        file.write("{} {} {} {}\n".format(2, 1, 0, v.shape[0]))

        for i in range(v.shape[0]):
            file.write("{}\n".format(i))

        for i in range(v.shape[0]):
            file.write("{} {} {}\n".format(v[i][0], v[i][1], v[i][2]))

        # for i in range(v.shape[0]):
        #     file.write("{} {}\n".format(v[i][0], v[i][1]))

        file.write("$EndNodes\n")
        file.write("$Elements\n")
        file.write("{} {} {} {}\n".format(1, l.shape[0], 0, l.shape[0] - 1))
        file.write("{} {} {} {}\n".format(2, 1, 1, l.shape[0]))

        for i in range(l.shape[0]):
            file.write("{} {} {}\n".format(i, l[i][0], l[i][1]))

        file.write("$EndElements\n")
        
def write_segment_msh_22(filename, v, l):
    with open(filename, "w") as file:
        file.write("$MeshFormat\n")
        file.write("2.2 0 8\n")
        file.write("$EndMeshFormat\n")
        file.write("$Nodes\n")
        file.write("{}\n".format(v.shape[0]))

        for i in range(v.shape[0]):
            file.write("{} {} {} {}\n".format(i, v[i][0], v[i][1], v[i][2]))

        file.write("$EndNodes\n")
        file.write("$Elements\n")
        file.write("{}\n".format(l.shape[0]))

        for i in range(l.shape[0]):
            file.write("{} {} {} {} {}\n".format(i, 1, 0, l[i][0], l[i][1]))

        file.write("$EndElements\n")
        

if __name__=="__main__":
    args = sys.argv

    if len(args) != 3:
        print("Error input format\nusage:\n    python convert_segment_obj2msh.py <input> <output>\n\n    e.g. python convert_segment_obj2msh.py input.obj output.msh\n")
        exit(0)
    
    v, l = read_segment_obj(args[1])
    write_segment_msh_22(args[2], v, l)

